// ------------------------------------------------------------------------------
// tb_dfdtmh_pkg.sv -- self-checking testbench for dfdtmh_pkg
//
// The package is the contract between RTL, CSR software and the paper, so this
// checks it against the DOCUMENTED layout (hard-coded here on purpose), not
// against itself:
//   * CSR addresses, contiguity, custom-RW range, no overlap with the SBTMH window
//   * FSM enum values / width
//   * DFD_CTRL, DFD_STATUS, DFD_SENS bit positions (the 21-bit status struct bug)
//   * is_sensitive(): directed cases + randomised against an independent bit-mask model
//   * parameter sanity the RTL depends on (powers of two, 32-bit word limits, ...)
// Static checks run at time 0; no clock needed.
// ------------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_pkg;
  import dfdtmh_pkg::*;

  int errs = 0;
  int checks = 0;

  task automatic expect_eq(input longint unsigned got, input longint unsigned exp, input string what);
    checks++;
    if (got !== exp) begin
      errs++;
      $display("FAIL: %s: got 0x%0h expected 0x%0h", what, got, exp);
    end
  endtask

  task automatic expect_true(input bit cond, input string what);
    checks++;
    if (!cond) begin
      errs++;
      $display("FAIL: %s", what);
    end
  endtask

  // independent reference for is_sensitive: build a 32-bit mask from raw 6-bit slots
  function automatic logic [31:0] ref_mask(input logic [6*NrSens-1:0] raw);
    logic [31:0] m;
    m = '0;
    for (int s = 0; s < NrSens; s++) begin
      if (raw[6*s+5]) m[raw[6*s+:5]] = 1'b1;
    end
    return m;
  endfunction

  function automatic bit is_pow2(input int unsigned v);
    return (v != 0) && ((v & (v - 1)) == 0);
  endfunction

  initial begin
    sens_table_t        tbl;
    dfdtmh_ctrl_t       ctrl;
    dfdtmh_status_t     st;
    sens_entry_t        ent;
    logic [31:0]        w32;
    logic [6*NrSens-1:0] raw;
    logic [11:0]        sbtmh_lo, sbtmh_hi;

    // -------------------------------------------------------------------
    // 1. CSR addresses (documented map)
    // -------------------------------------------------------------------
    $display("P1: CSR addresses");
    expect_eq(CSR_DFD_CTRL,   12'h7C8, "CSR_DFD_CTRL");
    expect_eq(CSR_DFD_STATUS, 12'h7C9, "CSR_DFD_STATUS");
    expect_eq(CSR_DFD_INTVL,  12'h7CA, "CSR_DFD_INTVL");
    expect_eq(CSR_DFD_SEL,    12'h7CB, "CSR_DFD_SEL");
    expect_eq(CSR_DFD_DATA,   12'h7CC, "CSR_DFD_DATA");
    expect_eq(CSR_DFD_SENS,   12'h7CD, "CSR_DFD_SENS");
    expect_eq(CSR_DFD_INFO,   12'h7CE, "CSR_DFD_INFO");
    begin
      logic [11:0] addrs [7];
      addrs = '{CSR_DFD_CTRL, CSR_DFD_STATUS, CSR_DFD_INTVL, CSR_DFD_SEL,
                CSR_DFD_DATA, CSR_DFD_SENS, CSR_DFD_INFO};
      sbtmh_lo = 12'h7C3; sbtmh_hi = 12'h7C7;     // SBTMH window (tmh_pkg.sv)
      for (int i = 0; i < 7; i++) begin
        for (int j = i + 1; j < 7; j++)
          expect_true(addrs[i] != addrs[j], $sformatf("CSR addresses %0d and %0d are distinct", i, j));
        // RISC-V CSR address rules: [11:10]!=2'b11 -> read/write, [9:8]==2'b11 -> machine mode
        expect_true(addrs[i][11:10] != 2'b11, $sformatf("CSR 0x%03h is read/write space", addrs[i]));
        expect_true(addrs[i][9:8] == 2'b11,   $sformatf("CSR 0x%03h is machine-mode space", addrs[i]));
        // 0x7C0-0x7FF = machine custom read/write
        expect_true(addrs[i] >= 12'h7C0 && addrs[i] <= 12'h7FF, $sformatf("CSR 0x%03h in custom RW range", addrs[i]));
        expect_true(!(addrs[i] >= sbtmh_lo && addrs[i] <= sbtmh_hi),
                    $sformatf("CSR 0x%03h does not overlap the SBTMH window", addrs[i]));
        if (i > 0) expect_eq(addrs[i], addrs[i-1] + 12'd1, $sformatf("CSR %0d contiguous", i));
      end
    end

    // -------------------------------------------------------------------
    // 2. FSM state encoding
    // -------------------------------------------------------------------
    $display("P2: FSM enum");
    expect_eq($bits(dfdtmh_state_e), 2, "state enum width");
    expect_eq(IDLE, 2'b00, "IDLE");
    expect_eq(RUN,  2'b01, "RUN");
    expect_eq(DONE, 2'b10, "DONE");
    expect_true(IDLE != RUN && RUN != DONE && IDLE != DONE, "states distinct");

    // -------------------------------------------------------------------
    // 3. DFD_CTRL layout: en[0] oneshot[1] mode[2] snapshot[3] clear[4] ack[5]
    // -------------------------------------------------------------------
    $display("P3: DFD_CTRL layout");
    expect_eq($bits(dfdtmh_ctrl_t), 6, "ctrl width");
    ctrl = 6'b000001; expect_true(ctrl.en && !ctrl.oneshot && !ctrl.mode && !ctrl.snapshot && !ctrl.clear && !ctrl.ack, "ctrl bit0 = en");
    ctrl = 6'b000010; expect_true(!ctrl.en && ctrl.oneshot && !ctrl.mode && !ctrl.snapshot && !ctrl.clear && !ctrl.ack, "ctrl bit1 = oneshot");
    ctrl = 6'b000100; expect_true(!ctrl.en && !ctrl.oneshot && ctrl.mode && !ctrl.snapshot && !ctrl.clear && !ctrl.ack, "ctrl bit2 = mode");
    ctrl = 6'b001000; expect_true(!ctrl.en && !ctrl.oneshot && !ctrl.mode && ctrl.snapshot && !ctrl.clear && !ctrl.ack, "ctrl bit3 = snapshot");
    ctrl = 6'b010000; expect_true(!ctrl.en && !ctrl.oneshot && !ctrl.mode && !ctrl.snapshot && ctrl.clear && !ctrl.ack, "ctrl bit4 = clear");
    ctrl = 6'b100000; expect_true(!ctrl.en && !ctrl.oneshot && !ctrl.mode && !ctrl.snapshot && !ctrl.clear && ctrl.ack, "ctrl bit5 = ack");

    // -------------------------------------------------------------------
    // 4. DFD_STATUS layout: state[1:0] new[2] overrun[3] sat[4] seq[31:16]
    // -------------------------------------------------------------------
    $display("P4: DFD_STATUS layout");
    expect_eq($bits(dfdtmh_status_t), 32, "status must be exactly 32 bits (CSR word)");
    st = '0; st.state = 2'b10;        expect_eq(32'(st), 32'h0000_0002, "status.state at [1:0]");
    st = '0; st.new_sample = 1'b1;    expect_eq(32'(st), 32'h0000_0004, "status.new_sample at [2]");
    st = '0; st.overrun = 1'b1;       expect_eq(32'(st), 32'h0000_0008, "status.overrun at [3]");
    st = '0; st.sat = 1'b1;           expect_eq(32'(st), 32'h0000_0010, "status.sat at [4]");
    st = '0; st.seq = 16'hABCD;       expect_eq(32'(st), 32'hABCD_0000, "status.seq at [31:16]");
    st = '0; st.seq = 16'h0001;       expect_eq(32'(st), 32'h0001_0000, "status.seq LSB is bit 16");
    st = '0; st.seq = 16'hFFFF; st.sat = 1; st.overrun = 1; st.new_sample = 1; st.state = 2'b11;
    expect_eq(32'(st), 32'hFFFF_001F, "status all-ones keeps bits [15:5] zero");
    st = '0; expect_eq(32'(st[15:5]), 0, "reserved bits read 0");

    // -------------------------------------------------------------------
    // 5. DFD_SENS layout: slot s at [6s +: 6] = {valid, reg[4:0]}
    // -------------------------------------------------------------------
    $display("P5: DFD_SENS layout");
    expect_eq($bits(sens_entry_t), 6, "sens entry width");
    expect_eq($bits(sens_table_t), 6 * NrSens, "sens table width");
    ent = '0; ent.valid = 1'b1;       expect_eq(6'(ent), 6'b100000, "entry.valid is bit 5");
    ent = '0; ent.reg_idx = 5'd31;    expect_eq(6'(ent), 6'b011111, "entry.reg_idx is [4:0]");
    for (int s = 0; s < NrSens; s++) begin
      logic [6*NrSens-1:0] exp_raw;
      tbl = '0;
      tbl[s].valid   = 1'b1;
      tbl[s].reg_idx = 5'(10 + s);
      exp_raw = '0;
      exp_raw[6*s+:6] = {1'b1, 5'(10 + s)};
      expect_eq(64'(tbl), 64'(exp_raw), $sformatf("slot %0d sits at [%0d +: 6]", s, 6*s));
    end
    // raw CSR write value -> table fields (what dfdtmh_csr will do)
    raw = '0;
    for (int s = 0; s < NrSens; s++) raw[6*s+:6] = {1'(s % 2 == 0), 5'(3 + 7*s)};
    tbl = sens_table_t'(raw);
    for (int s = 0; s < NrSens; s++) begin
      expect_eq(64'(tbl[s].valid),   64'(s % 2 == 0), $sformatf("raw->table valid[%0d]", s));
      expect_eq(64'(tbl[s].reg_idx), 64'(3 + 7*s),    $sformatf("raw->table idx[%0d]", s));
    end

    // -------------------------------------------------------------------
    // 6. is_sensitive(): directed
    // -------------------------------------------------------------------
    $display("P6: is_sensitive directed");
    tbl = '0;
    for (int r = 0; r < 32; r++) expect_true(!is_sensitive(5'(r), {>>{tbl}}), $sformatf("empty table: x%0d not sensitive", r));
    tbl[0].valid = 1'b1; tbl[0].reg_idx = 5'd5;
    expect_true( is_sensitive(5'd5,  {>>{tbl}}), "slot0 valid x5 -> x5 sensitive");
    expect_true(!is_sensitive(5'd6,  {>>{tbl}}), "x6 not sensitive");
    expect_true(!is_sensitive(5'd4,  {>>{tbl}}), "x4 not sensitive");
    tbl = '0; tbl[0].valid = 1'b0; tbl[0].reg_idx = 5'd7;
    expect_true(!is_sensitive(5'd7,  {>>{tbl}}), "INVALID slot must not make x7 sensitive");
    tbl = '0; tbl[NrSens-1].valid = 1'b1; tbl[NrSens-1].reg_idx = 5'd31;
    expect_true( is_sensitive(5'd31, {>>{tbl}}), "last slot is searched (x31)");
    tbl = '0; tbl[1].valid = 1'b1; tbl[1].reg_idx = 5'd0;
    expect_true( is_sensitive(5'd0,  {>>{tbl}}), "x0 configured valid -> reported sensitive (event_gen never touches x0)");
    tbl = '0; tbl[0].valid = 1'b1; tbl[0].reg_idx = 5'd9; tbl[2].valid = 1'b1; tbl[2].reg_idx = 5'd9;
    expect_true( is_sensitive(5'd9, tbl), "duplicate slots still sensitive");
    for (int s = 0; s < NrSens; s++) begin
      tbl = '0; tbl[s].valid = 1'b1; tbl[s].reg_idx = 5'(20 + s);
      for (int r = 0; r < 32; r++)
        expect_true(is_sensitive(5'(r), {>>{tbl}}) == (r == 20 + s), $sformatf("single slot %0d -> only x%0d", s, 20 + s));
    end
    tbl = '1;   // all slots valid, all idx 31
    for (int r = 0; r < 32; r++)
      expect_true(is_sensitive(5'(r), {>>{tbl}}) == (r == 31), $sformatf("all-ones table: only x31 (r=%0d)", r));

    // -------------------------------------------------------------------
    // 7. is_sensitive(): randomised vs independent mask model
    // -------------------------------------------------------------------
    $display("P7: is_sensitive random (20000 tables x 32 regs)");
    for (int t = 0; t < 20000; t++) begin
      raw = '0;
      for (int s = 0; s < NrSens; s++) raw[6*s+:6] = 6'($urandom());
      tbl = sens_table_t'(raw);
      w32 = ref_mask(raw);
      for (int r = 0; r < 32; r++) begin
        if (is_sensitive(5'(r), {>>{tbl}}) !== w32[r]) begin
          errs++;
          $display("FAIL: random table raw=0x%0h reg x%0d: got %b expected %b", raw, r, is_sensitive(5'(r), {>>{tbl}}), w32[r]);
        end
        checks++;
      end
    end

    // -------------------------------------------------------------------
    // 8. Parameter sanity the RTL depends on
    // -------------------------------------------------------------------
    $display("P8: parameter sanity");
    expect_eq(RegNum, 32, "RegNum is 32 (x0..x31, 5-bit register fields)");
    expect_true(NrSens >= 1, "NrSens >= 1");
    expect_true(NrSens * 6 <= 64, "DFD_SENS table fits one 64-bit CSR");
    if (NrSens * 6 > 32) $display("NOTE: NrSens*6 > 32 -> DFD_SENS does not fit an RV32 CSR (NrSens <= 5 for RV32)");
    expect_true(CntW >= 1 && CntW <= 32, "CntW in 1..32 (counter fits a 32-bit readout word)");
    expect_true(is_pow2(PcBlkW)  && PcBlkW  >= 2 && PcBlkW  <= 32, "PcBlkW power of two, <= 32 (bit-slice index, one word)");
    expect_true(is_pow2(MemBlkW) && MemBlkW >= 2 && MemBlkW <= 32, "MemBlkW power of two, <= 32");
    expect_eq(NumWords, NrSens + 4, "NumWords = NrSens + 4 (counters + reg/pc/mem bitmaps + retire)");
    expect_eq(SensIdxW, (NrSens > 1) ? $clog2(NrSens) : 1, "SensIdxW = clog2(NrSens)");
    expect_true((1 << SensIdxW) >= NrSens, "SensIdxW can index every slot");
    expect_true(NumWords <= 255, "NumWords fits the 8-bit DFD_INFO field");
    expect_true(NrSens <= 255, "NrSens fits the 8-bit DFD_INFO field");

    $display("checks run: %0d", checks);
    if (errs == 0) $display("ALL DFDTMH_PKG TESTS PASSED");
    else           $display("DFDTMH_PKG TESTS FAILED: %0d error(s)", errs);
    $finish;
  end
endmodule