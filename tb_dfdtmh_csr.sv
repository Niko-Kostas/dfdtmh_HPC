// ------------------------------------------------------------------------------
// tb_dfdtmh_csr.sv -- self-checking testbench for dfdtmh_csr
//
// Layer 1 (CD1..CD10): directed tests, expectations HAND-WRITTEN from the CSR map.
// Layer 2 (CR1)      : random bus traffic, every CSR and every output compared each
//                      cycle with a reference model.
//
// Reads are combinational in addr_i; a "peek" changes addr_i with we_i=0 for 0.1 ns,
// far inside one clock cycle, so peeks never disturb a pending write.
// ------------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_csr;
  import dfdtmh_pkg::*;

  localparam int unsigned DW = 64;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  logic [11:0]    addr;
  logic           we;
  logic [DW-1:0]  wdata, rdata;
  logic           hit;
  logic           en, oneshot, mode, force_roll, clear, ack;
  logic [31:0]    intvl;
  sens_table_t    sens;
  dfdtmh_state_e  state_in;
  logic           new_in, ovr_in, sat_in;
  logic [15:0]    seq_in;
  logic [31:0]    words [NumWords];

  dfdtmh_csr #(.CsrDataWidth(DW), .DefaultInterval(1000)) dut (
      .clk_i(clk), .rst_ni(rst_n),
      .addr_i(addr), .we_i(we), .data_i(wdata), .csr_rdata_o(rdata), .csr_addr_hit_o(hit),
      .en_o(en), .oneshot_o(oneshot), .mode_o(mode), .intvl_o(intvl),
      .force_roll_o(force_roll), .clear_o(clear), .ack_o(ack), .sens_o(sens),
      .state_i(state_in), .new_i(new_in), .ovr_i(ovr_in), .sat_i(sat_in), .seq_i(seq_in),
      .word_i(words)
  );

  int errs = 0;
  int checks = 0;

  task automatic fail(input string m);
    errs++;
    $display("[%0t] FAIL: %s", $time, m);
    if (errs >= 25) begin $display("too many errors"); $finish; end
  endtask

  task automatic expect64(input logic [63:0] got, input logic [63:0] exp, input string what);
    checks++;
    if (got !== exp) fail($sformatf("%s: got 0x%0h expected 0x%0h", what, got, exp));
  endtask

  // ---------------- bus helpers ---------------------------------------------------
  logic last_hit;                 // csr_addr_hit_o as seen during the last peek
  task automatic peek(input logic [11:0] a, output logic [63:0] v);
    logic [11:0] sa; logic swe;
    sa = addr; swe = we;
    addr = a; we = 1'b0;
    #0.1;
    v = rdata;
    last_hit = hit;
    addr = sa; we = swe;
    #0.1;
  endtask

  task automatic rd(input logic [11:0] a, input logic [63:0] exp, input string what);
    logic [63:0] v;
    peek(a, v);
    expect64(v, exp, what);
  endtask

  // one write cycle; returns the pulse outputs seen during that cycle
  task automatic wr(input logic [11:0] a, input logic [63:0] d,
                    output bit p_snap, output bit p_clear, output bit p_ack);
    addr = a; wdata = d; we = 1'b1;
    #1;
    p_snap = force_roll; p_clear = clear; p_ack = ack;
    @(posedge clk);
    @(negedge clk);
    we = 1'b0; wdata = '0; addr = '0;
    #1;
    checks++;
    if (force_roll || clear || ack) fail("pulse outputs must drop after the write cycle");
  endtask

  task automatic wr0(input logic [11:0] a, input logic [63:0] d);
    bit s, c, k;
    wr(a, d, s, c, k);
  endtask

  task automatic expect_pulses(input logic [11:0] a, input logic [63:0] d,
                               input bit e_snap, input bit e_clear, input bit e_ack, input string what);
    bit s, c, k;
    wr(a, d, s, c, k);
    checks++;
    if (s !== e_snap || c !== e_clear || k !== e_ack)
      fail($sformatf("%s: pulses snap=%b clear=%b ack=%b expected %b%b%b", what, s, c, k, e_snap, e_clear, e_ack));
  endtask

  // ---------------- reference model ------------------------------------------------
  bit          r_en, r_os, r_mode;
  longint unsigned r_intvl;
  int          r_sel;
  logic [DW-1:0] r_sens_raw;

  task automatic ref_reset();
    r_en = 0; r_os = 0; r_mode = 0; r_intvl = 1000; r_sel = 0; r_sens_raw = '0;
  endtask

  task automatic ref_write(input logic [11:0] a, input logic [63:0] d);
    if (a == CSR_DFD_CTRL)  begin r_en = d[0]; r_os = d[1]; r_mode = d[2]; end
    if (a == CSR_DFD_INTVL) r_intvl = (d[31:0] == 0) ? 1 : d[31:0];
    if (a == CSR_DFD_SEL)   r_sel = (d > 64'(NumWords) - 64'd1) ? (NumWords - 1) : int'(d[7:0]);
    if (a == CSR_DFD_SENS)  r_sens_raw = {'0, d[6*NrSens-1:0]};
  endtask

  function automatic logic [63:0] exp_status();
    logic [63:0] s;
    s = '0;
    s[1:0] = state_in; s[2] = new_in; s[3] = ovr_in; s[4] = sat_in; s[31:16] = seq_in;
    return s;
  endfunction

  task automatic check_all(input string tag);
    expect64({61'b0, r_mode, r_os, r_en}, {61'b0, mode, oneshot, en}, {tag, " ctrl outputs"});
    expect64(r_intvl, intvl, {tag, " intvl output"});
    for (int s = 0; s < NrSens; s++) begin
      expect64(r_sens_raw[6*s+:6], {58'b0, sens[s]}, $sformatf("%s sens slot %0d output", tag, s));
    end
    rd(CSR_DFD_CTRL,   {61'b0, r_mode, r_os, r_en}, {tag, " rd CTRL"});
    rd(CSR_DFD_STATUS, exp_status(),                {tag, " rd STATUS"});
    rd(CSR_DFD_INTVL,  r_intvl,                     {tag, " rd INTVL"});
    rd(CSR_DFD_SEL,    64'(r_sel),                  {tag, " rd SEL"});
    rd(CSR_DFD_DATA,   64'(words[r_sel]),           {tag, " rd DATA"});
    rd(CSR_DFD_SENS,   r_sens_raw,                  {tag, " rd SENS"});
    rd(CSR_DFD_INFO,   64'h0008_0401,               {tag, " rd INFO"});
  endtask

  task automatic set_words();
    for (int w = 0; w < NumWords; w++) words[w] = 32'hA5A5_0000 + 32'(w) * 32'h111 + 32'($urandom_range(0, 1) * 32'h1000_0000);
  endtask

  // ================================================================================
  initial begin
    bit s, c, k;
    logic [63:0] v;
    int hits;

    rst_n = 1'b0; addr = '0; we = 1'b0; wdata = '0;
    state_in = IDLE; new_in = 0; ovr_in = 0; sat_in = 0; seq_in = '0;
    for (int w = 0; w < NumWords; w++) words[w] = 32'hA5A5_0000 + 32'(w) * 32'h111;
    ref_reset();
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- CD1: reset values -------------------------------------------------------
    $display("CD1: reset values");
    rd(CSR_DFD_CTRL,   64'h0,           "CD1 CTRL");
    rd(CSR_DFD_INTVL,  64'd1000,        "CD1 INTVL default");
    rd(CSR_DFD_SEL,    64'h0,           "CD1 SEL");
    rd(CSR_DFD_SENS,   64'h0,           "CD1 SENS empty");
    rd(CSR_DFD_INFO,   64'h0008_0401,   "CD1 INFO = {NumWords=8, NrSens=4, version=1}");
    rd(CSR_DFD_DATA,   64'(words[0]),   "CD1 DATA shows word 0");
    checks += 5;
    if (en || oneshot || mode) fail("CD1 config outputs must reset to 0 (counting is an explicit action)");
    if (intvl !== 32'd1000)    fail("CD1 intvl_o");
    if (sens !== '0)           fail("CD1 sens_o");
    if (force_roll || clear || ack) fail("CD1 no pulses at reset");

    // ---- CD2: address decode ----------------------------------------------------------
    $display("CD2: address decode over all 4096 addresses");
    state_in = RUN; new_in = 1; seq_in = 16'hFFFF;
    hits = 0;
    for (int a = 0; a < 4096; a++) begin
      logic [63:0] vv;
      peek(12'(a), vv);
      checks++;
      if (last_hit !== ((a >= 12'h7C8) && (a <= 12'h7CE))) fail($sformatf("hit for 0x%03h = %b", a, last_hit));
      if (a < 12'h7C8 || a > 12'h7CE) expect64(vv, 64'h0, $sformatf("non-DFD address 0x%03h reads 0", a));
      if (last_hit) hits++;
    end
    expect64(hits, 7, "exactly 7 CSR addresses decode");
    state_in = IDLE; new_in = 0; seq_in = '0;
    // writes to neighbouring / unrelated addresses change nothing
    for (int a = 12'h7C0; a <= 12'h7D2; a++) begin
      if (a < 12'h7C8 || a > 12'h7CE) wr(12'(a), 64'hFFFF_FFFF_FFFF_FFFF, s, c, k);
    end
    wr0(12'h000, 64'hFFFF_FFFF_FFFF_FFFF);
    wr0(12'hFFF, 64'hFFFF_FFFF_FFFF_FFFF);
    rd(CSR_DFD_CTRL,  64'h0,    "CD2 CTRL untouched by foreign writes");
    rd(CSR_DFD_INTVL, 64'd1000, "CD2 INTVL untouched");
    rd(CSR_DFD_SENS,  64'h0,    "CD2 SENS untouched");
    checks++; if (clear || force_roll || ack) fail("CD2 foreign writes must not pulse");

    // ---- CD3: CTRL stored bits and pulse bits --------------------------------------------
    $display("CD3: CTRL");
    wr0(CSR_DFD_CTRL, 64'h0000_0007);
    rd(CSR_DFD_CTRL, 64'h7, "CD3 en|oneshot|mode stored");
    checks++; if (!en || !oneshot || !mode) fail("CD3 outputs follow");
    wr0(CSR_DFD_CTRL, 64'h0000_0001);
    rd(CSR_DFD_CTRL, 64'h1, "CD3 only en");
    wr0(CSR_DFD_CTRL, 64'h0000_0004);
    rd(CSR_DFD_CTRL, 64'h4, "CD3 only mode");
    wr0(CSR_DFD_CTRL, 64'h0000_0002);
    rd(CSR_DFD_CTRL, 64'h2, "CD3 only oneshot");
    expect_pulses(CSR_DFD_CTRL, 64'h08, 1, 0, 0, "CD3 snapshot only");
    expect_pulses(CSR_DFD_CTRL, 64'h10, 0, 1, 0, "CD3 clear only");
    expect_pulses(CSR_DFD_CTRL, 64'h20, 0, 0, 1, "CD3 ack only");
    expect_pulses(CSR_DFD_CTRL, 64'h38, 1, 1, 1, "CD3 all three pulses together");
    expect_pulses(CSR_DFD_CTRL, 64'h07, 0, 0, 0, "CD3 stored bits only: no pulses");
    rd(CSR_DFD_CTRL, 64'h7, "CD3 pulse bits are not stored (read as 0)");
    expect_pulses(CSR_DFD_CTRL, 64'hFFFF_FFFF_FFFF_FFC0, 0, 0, 0, "CD3 reserved bits [63:6] ignored");
    rd(CSR_DFD_CTRL, 64'h0, "CD3 reserved bits did not set anything");
    // pulses require we and the CTRL address
    addr = CSR_DFD_CTRL; wdata = 64'h38; we = 1'b0; #1;
    checks++; if (force_roll || clear || ack) fail("CD3 no pulse without we");
    addr = CSR_DFD_SEL; we = 1'b1; #1;
    checks++; if (force_roll || ack) fail("CD3 snapshot/ack bits on another CSR must not pulse");
    we = 1'b0; addr = '0; #1;
    wr0(CSR_DFD_CTRL, 64'h0);

    // ---- CD4: INTVL WARL ---------------------------------------------------------------------
    $display("CD4: INTVL");
    wr0(CSR_DFD_INTVL, 64'd0);
    rd(CSR_DFD_INTVL, 64'd1, "CD4 0 reads back as 1");
    checks++; if (intvl !== 32'd1) fail("CD4 intvl_o must be >= 1");
    wr0(CSR_DFD_INTVL, 64'd5);
    rd(CSR_DFD_INTVL, 64'd5, "CD4 5");
    wr0(CSR_DFD_INTVL, 64'hFFFF_FFFF);
    rd(CSR_DFD_INTVL, 64'hFFFF_FFFF, "CD4 max 32-bit value");
    wr0(CSR_DFD_INTVL, 64'h0000_0001_0000_0007);
    rd(CSR_DFD_INTVL, 64'd7, "CD4 only the low 32 bits are stored");
    wr0(CSR_DFD_INTVL, 64'h0000_0001_0000_0000);
    rd(CSR_DFD_INTVL, 64'd1, "CD4 low word 0 (upper bit set) still clamps to 1");

    // ---- CD5: SEL / DATA -----------------------------------------------------------------------
    $display("CD5: SEL and DATA");
    for (int w = 0; w < NumWords; w++) begin
      wr0(CSR_DFD_SEL, 64'(w));
      rd(CSR_DFD_SEL,  64'(w),          $sformatf("CD5 SEL=%0d", w));
      rd(CSR_DFD_DATA, 64'(words[w]),   $sformatf("CD5 DATA word %0d", w));
    end
    wr0(CSR_DFD_SEL, 64'(NumWords));
    rd(CSR_DFD_SEL, 64'(NumWords - 1), "CD5 SEL=NumWords clamps to NumWords-1");
    wr0(CSR_DFD_SEL, 64'd100);
    rd(CSR_DFD_SEL, 64'(NumWords - 1), "CD5 SEL=100 clamps");
    wr0(CSR_DFD_SEL, 64'hFFFF_FFFF_FFFF_FFFF);
    rd(CSR_DFD_SEL, 64'(NumWords - 1), "CD5 SEL=all-ones clamps");
    wr0(CSR_DFD_SEL, 64'h0000_0001_0000_0003);
    rd(CSR_DFD_SEL, 64'(NumWords - 1), "CD5 SEL with upper bits set compares as a large value -> clamps");
    wr0(CSR_DFD_SEL, 64'd2);
    wr0(CSR_DFD_DATA, 64'hDEAD_BEEF);
    rd(CSR_DFD_DATA, 64'(words[2]), "CD5 write to DATA ignored");
    rd(CSR_DFD_SEL, 64'd2,          "CD5 write to DATA did not touch SEL");
    // DATA follows the live word input
    words[2] = 32'h1234_5678;
    rd(CSR_DFD_DATA, 64'h1234_5678, "CD5 DATA tracks word_i combinationally");

    // ---- CD6: SENS -----------------------------------------------------------------------------
    $display("CD6: SENS");
    // slot0={1,x5}=0x25, slot1={1,x10}=0x2A, slot2={0,x7}=0x07, slot3={1,x31}=0x3F
    expect_pulses(CSR_DFD_SENS, 64'h0, 0, 1, 0, "CD6 any SENS write raises clear_o");
    expect_pulses(CSR_DFD_SENS, {40'b0, 6'h3F, 6'h07, 6'h2A, 6'h25} & 64'h00FF_FFFF, 0, 1, 0, "CD6 program table");
    rd(CSR_DFD_SENS, {40'b0, 6'h3F, 6'h07, 6'h2A, 6'h25}, "CD6 readback");
    checks += 8;
    if (!(sens[0].valid && sens[0].reg_idx == 5'd5))   fail("CD6 slot0 = x5 valid");
    if (!(sens[1].valid && sens[1].reg_idx == 5'd10))  fail("CD6 slot1 = x10 valid");
    if (!(!sens[2].valid && sens[2].reg_idx == 5'd7))  fail("CD6 slot2 = x7 invalid");
    if (!(sens[3].valid && sens[3].reg_idx == 5'd31))  fail("CD6 slot3 = x31 valid");
    expect_pulses(CSR_DFD_SENS, 64'hFFFF_FFFF_FFFF_FFFF, 0, 1, 0, "CD6 all-ones write");
    rd(CSR_DFD_SENS, 64'h00FF_FFFF, "CD6 only NrSens*6 bits are kept");
    wr0(CSR_DFD_SENS, 64'h0);
    rd(CSR_DFD_SENS, 64'h0, "CD6 cleared");

    // ---- CD7: STATUS layout and read-only ---------------------------------------------------------
    $display("CD7: STATUS / INFO read-only");
    state_in = RUN;  new_in = 1; ovr_in = 0; sat_in = 1; seq_in = 16'hBEEF;
    rd(CSR_DFD_STATUS, 64'hBEEF_0015, "CD7 {seq=BEEF, sat, new, state=RUN}");
    state_in = DONE; new_in = 0; ovr_in = 1; sat_in = 0; seq_in = 16'h0001;
    rd(CSR_DFD_STATUS, 64'h0001_000A, "CD7 {seq=1, overrun, state=DONE}");
    state_in = IDLE; new_in = 0; ovr_in = 0; sat_in = 0; seq_in = 16'h0;
    rd(CSR_DFD_STATUS, 64'h0, "CD7 all zero");
    state_in = DONE; new_in = 1; ovr_in = 1; sat_in = 1; seq_in = 16'hFFFF;
    wr0(CSR_DFD_STATUS, 64'h0);
    rd(CSR_DFD_STATUS, 64'hFFFF_001E, "CD7 write to STATUS ignored; bits [15:5] read 0");
    wr0(CSR_DFD_INFO, 64'hFFFF_FFFF);
    rd(CSR_DFD_INFO, 64'h0008_0401, "CD7 write to INFO ignored");
    state_in = IDLE; new_in = 0; ovr_in = 0; sat_in = 0; seq_in = 16'h0;

    // ---- CD8: no write without we -------------------------------------------------------------------
    $display("CD8: we=0 never writes");
    wr0(CSR_DFD_CTRL, 64'h5); wr0(CSR_DFD_INTVL, 64'd77); wr0(CSR_DFD_SEL, 64'd3);
    wr0(CSR_DFD_SENS, 64'h0000_0000_0000_0A25);
    for (int a = 12'h7C8; a <= 12'h7CE; a++) begin
      addr = 12'(a); wdata = 64'hFFFF_FFFF_FFFF_FFFF; we = 1'b0;
      @(posedge clk); @(negedge clk);
    end
    we = 1'b0; addr = '0;
    rd(CSR_DFD_CTRL, 64'h5,    "CD8 CTRL");
    rd(CSR_DFD_INTVL, 64'd77,  "CD8 INTVL");
    rd(CSR_DFD_SEL, 64'd3,     "CD8 SEL");
    rd(CSR_DFD_SENS, 64'h0A25, "CD8 SENS");

    // ---- CD9: asynchronous reset ------------------------------------------------------------------
    $display("CD9: async reset");
    #1 rst_n = 1'b0; #1;
    ref_reset();
    rd(CSR_DFD_CTRL, 64'h0, "CD9 CTRL reset"); rd(CSR_DFD_INTVL, 64'd1000, "CD9 INTVL reset");
    rd(CSR_DFD_SEL, 64'h0, "CD9 SEL reset");   rd(CSR_DFD_SENS, 64'h0, "CD9 SENS reset");
    @(negedge clk); rst_n = 1'b1; @(negedge clk);

    // ---- CR1: random ------------------------------------------------------------------------------------
    $display("CR1: random bus traffic (80000 cycles)");
    wr0(CSR_DFD_CTRL, 64'h0); ref_reset(); ref_write(CSR_DFD_INTVL, 64'd1000);
    for (int cyc = 0; cyc < 80000; cyc++) begin
      logic [11:0] a;
      logic [63:0] d;
      bit do_we;
      bit e_snap, e_clear, e_ack;
      int kind;
      kind = $urandom_range(0, 9);
      if (kind < 8) a = 12'h7C8 + 12'($urandom_range(0, 6));
      else if (kind == 8) a = 12'h7C0 + 12'($urandom_range(0, 15));
      else a = 12'($urandom());
      case ($urandom_range(0, 4))
        0: d = 64'($urandom());
        1: d = {32'($urandom()), 32'($urandom())};
        2: d = 64'($urandom_range(0, 12));
        3: d = 64'h0;
        default: d = 64'hFFFF_FFFF_FFFF_FFFF;
      endcase
      do_we = ($urandom_range(0, 2) != 0);
      if ($urandom_range(0, 7) == 0) begin
        state_in = dfdtmh_state_e'($urandom_range(0, 2)); new_in = 1'($urandom()); ovr_in = 1'($urandom());
        sat_in = 1'($urandom()); seq_in = 16'($urandom());
        set_words();
      end
      addr = a; wdata = d; we = do_we;
      e_snap  = do_we && (a == CSR_DFD_CTRL) && d[3];
      e_clear = do_we && (((a == CSR_DFD_CTRL) && d[4]) || (a == CSR_DFD_SENS));
      e_ack   = do_we && (a == CSR_DFD_CTRL) && d[5];
      #1;
      checks += 3;
      if (force_roll !== e_snap)  fail($sformatf("random: snapshot pulse %b expected %b (addr 0x%03h data 0x%0h)", force_roll, e_snap, a, d));
      if (clear !== e_clear)      fail($sformatf("random: clear pulse %b expected %b (addr 0x%03h data 0x%0h)", clear, e_clear, a, d));
      if (ack !== e_ack)          fail($sformatf("random: ack pulse %b expected %b (addr 0x%03h data 0x%0h)", ack, e_ack, a, d));
      checks++;
      if (hit !== ((a >= 12'h7C8) && (a <= 12'h7CE))) fail($sformatf("random: hit for 0x%03h", a));
      @(posedge clk);
      @(negedge clk);
      if (do_we) ref_write(a, d);
      we = 1'b0;
      check_all("random");
    end

    $display("checks run: %0d", checks);
    if (errs == 0) $display("ALL DFDTMH_CSR TESTS PASSED");
    else           $display("DFDTMH_CSR TESTS FAILED: %0d error(s)", errs);
    $finish;
  end
endmodule
