// -----------------------------------------------------------------------------
// tb_dfdtmh_unit.sv -- standalone self-checking testbench for cva6_dfdtmh_unit
//
// Layer 1: directed tests with HAND-COMPUTED expected values (D1..D11). These do
//          not depend on the reference model below.
// Layer 2: constrained-random regression (R1) checked EVERY CYCLE against a
//          behavioural reference model written from the spec, covering STATUS,
//          CTRL, INTVL, SEL, SENS and the currently selected DATA word, plus the
//          interval_done_o strobe. Coverage counters make the run fail if the
//          random phase never reached the interesting corners.
//
// Timing convention: inputs change right after a negedge, DUT samples at the
// next posedge. step() = evaluate reference, sample strobe just before the
// posedge, advance one clock, compare, then return the bus to idle.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_unit;
  import ariane_pkg::*;
  import dfdtmh_pkg::*;
  // NOTE: this file targets the v2 design: register touched at most once per cycle,
  // blocks = (addr >> shift) mod NumBlocks, counters CntW bits wide (package parameter).

  // ---- DUT configuration -------------------------------------------------
  localparam int unsigned NP   = 2;   // commit ports
  localparam int unsigned NS   = NrSens;
  localparam int unsigned NW   = NumWords;
  localparam int unsigned MAXC = (1 << CntW) - 1;

  typedef struct packed {
    logic [63:0] pc;
    fu_t         fu;
    fu_op        op;
    logic [4:0]  rs1, rs2, rd;
    logic        use_zimm;
  } e_t;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  e_t   [NP-1:0] ci;
  logic [NP-1:0] ack;
  logic [55:0]   maddr;
  logic [11:0]   caddr;
  logic          cwe;
  logic [63:0]   cwd, crd;
  logic          chit, done;

  cva6_dfdtmh_unit #(
      .NR_COMMIT_PORTS(NP), .scoreboard_entry_t(e_t), .CsrDataWidth(64),
      .MemAddrWidth(56), .DefaultInterval(1000)
  ) dut (
      .clk_i(clk), .rst_ni(rst_n), .commit_instr_i(ci), .commit_ack_i(ack),
      .mem_addr_i(maddr), .addr_i(caddr), .we_i(cwe), .data_i(cwd),
      .csr_rdata_o(crd), .csr_addr_hit_o(chit), .interval_done_o(done)
  );
  // ------------------------------------------------------------------
  // VCD waveform
  // ------------------------------------------------------------------
  initial begin
    $dumpfile("cva6_dfdtmh_unit.vcd");
    $dumpvars(0, tb_dfdtmh_unit);
  end
  // ======================================================================
  // Reference model (state + per-cycle step)
  // ======================================================================
  int          r_state;
  bit          r_en, r_os, r_mode;
  int unsigned r_intvl;
  int          r_sel;
  bit          r_sv [NS];
  bit [4:0]    r_si [NS];
  int unsigned r_icnt;
  bit          r_new, r_ovr;
  int          r_seq;
  longint      l_cnt [NS];  bit [31:0] l_rmap; bit [PcBlkW-1:0] l_pmap; bit [MemBlkW-1:0] l_mmap; longint l_ret; bit l_sat;
  longint      s_cnt [NS];  bit [31:0] s_rmap; bit [PcBlkW-1:0] s_pmap; bit [MemBlkW-1:0] s_mmap; longint s_ret; bit s_sat;
  bit          x_roll;
  logic        done_s;                      // interval_done_o as seen at the last posedge
  always @(posedge clk) done_s <= done;

  int errs = 0;
  // coverage of the random phase
  int cv_roll, cv_ovr, cv_done, cv_sat, cv_snap_idle, cv_clear, cv_instr_mode, cv_dual_same, cv_ack;

  function automatic bit [31:0] s_word(int w);
    if (w < NS)           return 32'(s_cnt[w]);
    else if (w == NS)     return s_rmap;
    else if (w == NS + 1) return 32'(s_pmap);
    else if (w == NS + 2) return 32'(s_mmap);
    else                  return 32'(s_ret);
  endfunction

  function automatic bit is_sens(int r);
    for (int s = 0; s < NS; s++) if (r_sv[s] && int'(r_si[s]) == r) return 1'b1;
    return 1'b0;
  endfunction

  task automatic ref_reset();
    r_state = 0; r_en = 0; r_os = 0; r_mode = 0; r_intvl = 1000; r_sel = 0;
    r_icnt = 0; r_new = 0; r_ovr = 0; r_seq = 0;
    for (int s = 0; s < NS; s++) begin r_sv[s] = 0; r_si[s] = 0; l_cnt[s] = 0; s_cnt[s] = 0; end
    l_rmap = 0; l_pmap = 0; l_mmap = 0; l_ret = 0; l_sat = 0;
    s_rmap = 0; s_pmap = 0; s_mmap = 0; s_ret = 0; s_sat = 0;
    x_roll = 0;
  endtask

  task automatic ref_step();
    bit w_ctrl, snap, clr, ackp, run, expired, roll, memop, old_new, same_dual;
    int nstate, tick, v;
    int e_rc [32];
    bit [PcBlkW-1:0]  e_pc;
    bit [MemBlkW-1:0] e_mem;
    int e_ret;

    w_ctrl = cwe && (caddr == CSR_DFD_CTRL);
    snap   = w_ctrl && cwd[3];
    clr    = (w_ctrl && cwd[4]) || (cwe && caddr == CSR_DFD_SENS);
    ackp   = w_ctrl && cwd[5];

    // ---- events from the retiring instructions -------------------------
    for (int r = 0; r < 32; r++) e_rc[r] = 0;
    e_pc = 0; e_mem = 0; e_ret = 0; memop = 0; same_dual = 0;
    for (int p = 0; p < NP; p++) begin
      if (ack[p]) begin
        bit t [32];
        bit u1, u2, ud;
        for (int r = 0; r < 32; r++) t[r] = 0;
        u1 = 0; u2 = 0; ud = 0;
        e_ret++;
        case (ci[p].fu)
          ALU, MULT, CTRL_FLOW: begin u1 = 1; u2 = 1; ud = 1; end
          LOAD:  begin u1 = 1; u2 = 1; ud = !(ci[p].op inside {FLD, FLW, FLH, FLB}); end
          STORE: begin u1 = 1; ud = 1; u2 = !(ci[p].op inside {FSD, FSW, FSH, FSB}); end
          CSR:   begin u2 = 1; ud = 1; u1 = !ci[p].use_zimm; end
          default: ;
        endcase
        if (u1) t[ci[p].rs1] = 1;
        if (u2) t[ci[p].rs2] = 1;
        if (ud) t[ci[p].rd]  = 1;
        for (int r = 1; r < 32; r++) if (t[r]) begin
          if (e_rc[r] == 1) same_dual = 1;     // already touched by the other port this cycle
          e_rc[r] = 1;                         // v2: a register counts once per cycle
        end
        if (ci[p].fu == CTRL_FLOW) e_pc[int'((ci[p].pc >> PcShift) % PcBlkW)] = 1;
        if (ci[p].fu == LOAD || ci[p].fu == STORE) memop = 1;
      end
    end
    if (memop) e_mem[int'((maddr >> MemShift) % MemBlkW)] = 1;

    // ---- FSM -----------------------------------------------------------
    case (r_state)
      0: nstate = r_en ? 1 : 0;
      1: nstate = r_en ? 1 : 0;
      default: nstate = ackp ? (r_en ? 1 : 0) : 2;
    endcase
    if (clr) nstate = r_en ? 1 : 0;
    run = (nstate == 1);

    // ---- accumulate ------------------------------------------------------
    if (run) begin
      for (int s = 0; s < NS; s++) begin
        if (r_sv[s]) begin
          v = l_cnt[s] + e_rc[r_si[s]];
          if (v > MAXC) begin v = MAXC; l_sat = 1; end
          l_cnt[s] = v;
        end
      end
      for (int r = 0; r < 32; r++) if (e_rc[r] > 0 && !is_sens(r)) l_rmap[r] = 1'b1;
      l_pmap |= e_pc;
      l_mmap |= e_mem;
      v = l_ret + e_ret;
      if (v > MAXC) begin v = MAXC; l_sat = 1; end
      l_ret = v;
    end

    // ---- interval expiry / roll -----------------------------------------
    tick    = r_mode ? e_ret : 1;
    expired = run && ((r_icnt + tick) >= r_intvl);
    roll    = !clr && (snap || expired);
    if (clr || roll) r_icnt = 0; else if (run) r_icnt = r_icnt + tick;
    if (roll && r_os && run) nstate = 2;

    old_new = r_new;
    if (clr) begin
      r_new = 0; r_ovr = 0; r_seq = 0;
      for (int s = 0; s < NS; s++) begin l_cnt[s] = 0; s_cnt[s] = 0; end
      l_rmap = 0; l_pmap = 0; l_mmap = 0; l_ret = 0; l_sat = 0;
      s_rmap = 0; s_pmap = 0; s_mmap = 0; s_ret = 0; s_sat = 0;
    end else begin
      if (ackp) begin r_new = 0; r_ovr = 0; end
      if (roll) begin
        if (old_new && !ackp) begin r_ovr = 1; cv_ovr++; end
        r_new = 1;
        r_seq = (r_seq + 1) & 32'h0000_FFFF;
        for (int s = 0; s < NS; s++) begin s_cnt[s] = l_cnt[s]; l_cnt[s] = 0; end
        s_rmap = l_rmap; l_rmap = 0;
        s_pmap = l_pmap; l_pmap = 0;
        s_mmap = l_mmap; l_mmap = 0;
        s_ret  = l_ret;  l_ret  = 0;
        s_sat  = l_sat;  l_sat  = 0;
        cv_roll++;
        if (s_sat) cv_sat++;
        if (nstate == 2 && r_state != 2) cv_done++;
        if (snap && r_state == 0) cv_snap_idle++;
      end
    end
    if (clr) cv_clear++;
    if (ackp) cv_ack++;
    if (r_mode && run) cv_instr_mode++;
    if (same_dual) cv_dual_same++;
    x_roll  = roll;
    r_state = nstate;

    // ---- CSR register updates (visible from the next cycle) -------------
    if (cwe) begin
      if (caddr == CSR_DFD_CTRL) begin r_en = cwd[0]; r_os = cwd[1]; r_mode = cwd[2]; end
      if (caddr == CSR_DFD_INTVL) r_intvl = (cwd[31:0] == 0) ? 1 : cwd[31:0];
      if (caddr == CSR_DFD_SEL)   r_sel   = (cwd > 64'(NW) - 64'd1) ? int'(NW - 1) : int'(cwd[7:0]);
      if (caddr == CSR_DFD_SENS) begin
        for (int s = 0; s < NS; s++) begin r_sv[s] = cwd[6*s+5]; r_si[s] = cwd[6*s+:5]; end
      end
    end
  endtask

  // ======================================================================
  // Bus helpers
  // ======================================================================
  task automatic peek(input logic [11:0] a, output logic [63:0] v);
    caddr = a; cwe = 1'b0;
    #0.1;                                   // combinational settle; stays far inside one cycle
    v = crd;
  endtask

  task automatic fail(input string msg);
    errs++;
    $display("[%0t] FAIL: %s", $time, msg);
    if (errs >= 25) begin
      $display("DFDTMH TESTS FAILED: too many errors, aborting");
      $finish;
    end
  endtask

  task automatic check_csr(input logic [11:0] a, input logic [63:0] exp, input string what);
    logic [63:0] v;
    peek(a, v);
    if (v !== exp) fail($sformatf("%s: CSR 0x%03h read 0x%0h expected 0x%0h", what, a, v, exp));
  endtask

  function automatic logic [63:0] exp_status();
    logic [63:0] s;
    s = '0;
    s[1:0] = r_state[1:0];
    s[2] = r_new; s[3] = r_ovr; s[4] = s_sat;
    s[16+:16] = r_seq[15:0];
    return s;
  endfunction

  // compare every readable CSR against the reference model
  task automatic check_all(input string tag);
    logic [63:0] sens;
    sens = '0;
    for (int s = 0; s < NS; s++) sens[6*s+:6] = {r_sv[s], r_si[s]};
    check_csr(CSR_DFD_STATUS, exp_status(),                       {tag, " STATUS"});
    check_csr(CSR_DFD_CTRL,   64'({r_mode, r_os, r_en}),          {tag, " CTRL"});
    check_csr(CSR_DFD_INTVL,  64'(r_intvl),                       {tag, " INTVL"});
    check_csr(CSR_DFD_SEL,    64'(r_sel),                         {tag, " SEL"});
    check_csr(CSR_DFD_SENS,   sens,                               {tag, " SENS"});
    check_csr(CSR_DFD_DATA,   64'(s_word(r_sel)),                 {tag, " DATA"});
  endtask

  // one clock: reference -> strobe check -> clock -> compare -> idle bus
  task automatic step();
    ref_step();
    @(posedge clk);
    @(negedge clk);
    if (done_s !== x_roll) fail($sformatf("interval_done_o=%b expected %b", done_s, x_roll));
    check_all("cycle");
    ack = '0; cwe = 1'b0; caddr = '0; cwd = '0;
  endtask

  task automatic drive(input int p, input fu_t fu, input fu_op op,
                       input int rs1, input int rs2, input int rd,
                       input logic [63:0] pc, input bit zimm = 1'b0);
    ci[p].fu = fu; ci[p].op = op;
    ci[p].rs1 = 5'(rs1); ci[p].rs2 = 5'(rs2); ci[p].rd = 5'(rd);
    ci[p].pc = pc; ci[p].use_zimm = zimm;
    ack[p] = 1'b1;
  endtask

  task automatic csr_wr(input logic [11:0] a, input logic [63:0] d);
    caddr = a; cwd = d; cwe = 1'b1;
    step();
  endtask

  task automatic idle(input int n);
    for (int i = 0; i < n; i++) step();
  endtask

  // absolute (hand-computed) expectations
  task automatic expect_word(input int w, input logic [63:0] exp, input string what);
    csr_wr(CSR_DFD_SEL, 64'(w));
    check_csr(CSR_DFD_DATA, exp, what);
  endtask

  task automatic expect_status(input int st, input bit nw, input bit ov, input bit sa,
                               input int sq, input string what);
    logic [63:0] e;
    e = '0;
    e[1:0] = 2'(st); e[2] = nw; e[3] = ov; e[4] = sa;
    e[16+:16] = 16'(sq);
    check_csr(CSR_DFD_STATUS, e, what);
  endtask

  function automatic logic [63:0] sens_word(input int v0, input int i0, input int v1, input int i1,
                                            input int v2, input int i2, input int v3, input int i3);
    return 64'({v3[0], 5'(i3), v2[0], 5'(i2), v1[0], 5'(i1), v0[0], 5'(i0)});
  endfunction

  function automatic logic [63:0] ctrl(input bit en, input bit os, input bit md,
                                       input bit snap, input bit clr, input bit ak);
    return 64'({ak, clr, snap, md, os, en});
  endfunction

  // ======================================================================
  // Random-phase helpers
  // ======================================================================
  function automatic fu_t rnd_fu();
    case ($urandom_range(0, 9))
      0, 1:    return ALU;
      2:       return MULT;
      3:       return CTRL_FLOW;
      4, 5:    return LOAD;
      6:       return STORE;
      7:       return CSR;
      8:       return FPU;
      default: return ACCEL;
    endcase
  endfunction

  function automatic fu_op rnd_op(input fu_t fu);
    case (fu)
      LOAD:  case ($urandom_range(0, 4)) 0: return FLD; 1: return FLW; 2: return FLH; 3: return FLB; default: return LD; endcase
      STORE: case ($urandom_range(0, 4)) 0: return FSD; 1: return FSW; 2: return FSH; 3: return FSB; default: return SD; endcase
      CSR:   return ($urandom_range(0, 1) == 1) ? CSR_WRITE : CSR_READ;
      default: return ADD;
    endcase
  endfunction

  task automatic rnd_commit();
    for (int p = 0; p < NP; p++) begin
      if ($urandom_range(0, 9) < 7) begin
        fu_t f;
        f = rnd_fu();
        // registers 0..11 are heavily reused so sensitive slots get hits
        drive(p, f, rnd_op(f), $urandom_range(0, 11), $urandom_range(0, 11),
              $urandom_range(0, 11), 64'($urandom()), 1'($urandom_range(0, 1)));
        if ($urandom_range(0, 7) == 0) begin  // occasional full-range register
          ci[p].rs1 = 5'($urandom_range(0, 31));
          ci[p].rd  = 5'($urandom_range(0, 31));
        end
      end
    end
    maddr = 56'($urandom()) << 8 | 56'($urandom());
  endtask

  task automatic rnd_csr();
    int k;
    int unsigned iv [8];
    iv = '{0, 1, 2, 3, 5, 8, 13, 40};
    k = $urandom_range(0, 99);
    if (k < 40) begin                                   // CTRL
      logic [63:0] d;
      d = '0;
      d[0] = ($urandom_range(0, 9) < 8);
      d[1] = ($urandom_range(0, 4) == 0);
      d[2] = ($urandom_range(0, 9) < 3);
      d[3] = ($urandom_range(0, 9) < 1);
      d[4] = ($urandom_range(0, 19) == 0);
      d[5] = ($urandom_range(0, 9) < 3);
      caddr = CSR_DFD_CTRL; cwd = d; cwe = 1'b1;
    end else if (k < 55) begin                          // INTVL (incl. 0 => WARL, and a rare long one)
      caddr = CSR_DFD_INTVL;
      cwd   = ($urandom_range(0, 12) == 0) ? 64'(300) + 64'($urandom_range(0, 200)) : 64'(iv[$urandom_range(0, 7)]);
      cwe   = 1'b1;
    end else if (k < 85) begin                          // SEL (incl. out of range => WARL)
      caddr = CSR_DFD_SEL; cwd = 64'($urandom_range(0, 12)); cwe = 1'b1;
    end else if (k < 92) begin                          // SENS (clears everything)
      logic [63:0] d;
      d = '0;
      for (int s = 0; s < NS; s++) d[6*s+:6] = {1'($urandom_range(0, 3) != 0), 5'($urandom_range(0, 11))};
      caddr = CSR_DFD_SENS; cwd = d; cwe = 1'b1;
    end else begin                                      // write to a read-only CSR must be ignored
      caddr = ($urandom_range(0, 1) == 1) ? CSR_DFD_DATA : (($urandom_range(0, 1) == 1) ? CSR_DFD_STATUS : CSR_DFD_INFO);
      cwd = 64'($urandom()); cwe = 1'b1;
    end
  endtask

  // ======================================================================
  // Main sequence
  // ======================================================================
  initial begin
    rst_n = 1'b0; ci = '0; ack = '0; maddr = '0; caddr = '0; cwe = 1'b0; cwd = '0;
    cv_roll = 0; cv_ovr = 0; cv_done = 0; cv_sat = 0; cv_snap_idle = 0; cv_clear = 0;
    cv_instr_mode = 0; cv_dual_same = 0; cv_ack = 0;
    ref_reset();
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ------------------------------------------------------------------
    // D1: reset values and INFO
    // ------------------------------------------------------------------
    $display("D1: reset values / INFO");
    check_csr(CSR_DFD_INFO,   64'h0008_0401, "INFO = {NumWords=8, NrSens=4, ver=1}");
    check_csr(CSR_DFD_CTRL,   64'h0, "CTRL resets disabled");
    check_csr(CSR_DFD_INTVL,  64'd1000, "INTVL default");
    check_csr(CSR_DFD_SENS,   64'h0, "SENS resets empty");
    expect_status(0, 0, 0, 0, 0, "STATUS after reset");
    // CSR hit decode covers exactly the window
    caddr = 12'h7C8; #0.1; if (!chit) fail("0x7C8 must hit");
    caddr = 12'h7CE; #0.1; if (!chit) fail("0x7CE must hit");
    caddr = 12'h7C7; #0.1; if (chit)  fail("0x7C7 (SBTMH window) must NOT hit");
    caddr = 12'h7CF; #0.1; if (chit)  fail("0x7CF must NOT hit");
    caddr = '0;

    // ------------------------------------------------------------------
    // D2: WARL behaviour
    // ------------------------------------------------------------------
    $display("D2: WARL");
    csr_wr(CSR_DFD_INTVL, 64'd0);
    check_csr(CSR_DFD_INTVL, 64'd1, "INTVL 0 clamps to 1");
    csr_wr(CSR_DFD_SEL, 64'd99);
    check_csr(CSR_DFD_SEL, 64'(NW) - 64'd1, "SEL clamps to NumWords-1");
    csr_wr(CSR_DFD_INFO, 64'hFFFF_FFFF);
    check_csr(CSR_DFD_INFO, 64'h0008_0401, "INFO write ignored");

    // ------------------------------------------------------------------
    // D3: hand-computed interval (cycle mode, 8 cycles, one-shot)
    //     sensitive: slot0 = x5, slot1 = x10
    // ------------------------------------------------------------------
    $display("D3: hand-computed interval");
    csr_wr(CSR_DFD_SENS, sens_word(1, 5, 1, 10, 0, 0, 0, 0));
    csr_wr(CSR_DFD_INTVL, 64'd8);
    csr_wr(CSR_DFD_CTRL, ctrl(1, 1, 0, 0, 0, 0));     // EN | ONESHOT
    // c0: ADD x5 <- x6, x0            -> x5+1, x6 touched
    drive(0, ALU, ADD, 6, 0, 5, 64'h10);                                   step();
    // c1: BRANCH x5,x7 (pc 0x20) + ADD x5 <- x5,x5 (counts once)  -> x5+2, x7 touched, pc block 2
    drive(0, CTRL_FLOW, BRANCH, 5, 7, 0, 64'h20);
    drive(1, ALU, ADD, 5, 5, 5, 64'h24);                                   step();
    // c2: LD x11 <- [x10]  mem 0x1230 -> x10+1, x11 touched, mem block 3
    maddr = 56'h1230; drive(0, LOAD, LD, 10, 0, 11, 64'h30);               step();
    // c3: FLD f5 <- [x12] mem 0x1240 -> x12 touched, rd is FP (x5 NOT counted), mem block 4
    //     + port1 ADD x0,x0,x0 -> retires, touches nothing
    maddr = 56'h1240; drive(0, LOAD, FLD, 12, 0, 5, 64'h34);
    drive(1, ALU, ADD, 0, 0, 0, 64'h38);                                   step();
    // c4: FSD f10 -> [x13] mem 0x1250: rs2 is FP (x10 NOT counted), x13 touched, mem block 5
    maddr = 56'h1250; drive(0, STORE, FSD, 13, 10, 0, 64'h3c);             step();
    // c5: CSRRWI zimm=10 (rs1 is immediate: x10 NOT counted) rd=x14
    //     + CSRR rs1=x10 rd=x15 -> x10+1
    drive(0, CSR, CSR_WRITE, 10, 0, 14, 64'h40, 1'b1);
    drive(1, CSR, CSR_READ, 10, 0, 15, 64'h44, 1'b0);                      step();
    // c6: FPU op touching x5 (ignored, still retires) + MULT x10,x10,x10 -> x10+1 once
    drive(0, FPU, ADD, 5, 5, 5, 64'h48);
    drive(1, MULT, MUL, 10, 10, 10, 64'h4c);                               step();
    // c7: idle -> interval ends this cycle
    step();

    expect_status(2, 1, 0, 0, 1, "D3 after interval: DONE, NEW, seq=1");
    expect_word(0, 2,        "D3 x5 count (c0, c1 once; FLD rd and FPU masked)");
    expect_word(1, 3,        "D3 x10 count");
    expect_word(2, 64'h0,    "D3 unused slot 2");
    expect_word(3, 64'h0,    "D3 unused slot 3");
    expect_word(4, 64'hF8C0, "D3 non-sensitive reg bitmap (x6,x7,x11..x15)");
    expect_word(5, 64'h0100, "D3 PC bitmap (branch pc 0x20 -> block (0x20>>2) mod 16 = 8)");
    expect_word(6, 64'h0038, "D3 mem bitmap (blocks 3,4,5)");
    expect_word(7, 64'd11,   "D3 retired count");

    // DONE must hold: further activity is not accumulated until ACK
    drive(0, ALU, ADD, 5, 5, 5, 64'h80);
    drive(1, ALU, ADD, 5, 5, 5, 64'h84);                                   step();
    expect_status(2, 1, 0, 0, 1, "D3 still DONE");
    csr_wr(CSR_DFD_CTRL, ctrl(1, 1, 0, 0, 0, 1));     // ACK (EN, ONESHOT kept)
    expect_status(1, 0, 0, 0, 1, "D3 ACK -> RUN, NEW cleared");
    idle(7);                                            // ACK cycle + 7 idle = 8-cycle interval
    expect_status(2, 1, 0, 0, 2, "D3 second interval: DONE, seq=2");
    expect_word(0, 0, "D3b x5 zero (nothing leaked from DONE)");
    expect_word(7, 0, "D3b retired zero");

    // ------------------------------------------------------------------
    // D4: free-running intervals, overrun, ack
    // ------------------------------------------------------------------
    $display("D4: overrun / ack");
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 0, 1, 0));     // CLEAR, disabled
    idle(1);                                            // STATUS.state is registered: lags EN by 1 cycle
    expect_status(0, 0, 0, 0, 0, "D4 after CLEAR");
    csr_wr(CSR_DFD_INTVL, 64'd4);
    csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 0, 0, 0));     // free running
    idle(3);                                            // 4th active cycle rolls below
    idle(1);
    expect_status(1, 1, 0, 0, 1, "D4 first roll, NEW");
    idle(3);
    expect_status(1, 1, 0, 0, 1, "D4 not yet second roll");   // check consumed 1 cycle (csr_wr inside)
    idle(8);
    check_all("D4");
    // at least two more rolls happened with NEW never acked -> OVERRUN
    begin
      logic [63:0] st;
      peek(CSR_DFD_STATUS, st);
      if (!st[3]) fail("D4 OVERRUN must be set after unacked rolls");
    end
    csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 0, 0, 1));     // ACK
    begin
      logic [63:0] st;
      peek(CSR_DFD_STATUS, st);
      if (st[3]) fail("D4 OVERRUN must clear on ACK");
    end

    // ------------------------------------------------------------------
    // D5: instruction-count mode with dual commit (overshoot by 1)
    // ------------------------------------------------------------------
    $display("D5: instruction mode");
    csr_wr(CSR_DFD_SENS, sens_word(1, 5, 0, 0, 0, 0, 0, 0));     // clears
    csr_wr(CSR_DFD_INTVL, 64'd5);
    // Reconfigure + CLEAR in one write: the interval counter must not carry ticks
    // counted in the old unit (cycles) into the new one (instructions).
    csr_wr(CSR_DFD_CTRL, ctrl(1, 1, 1, 0, 1, 0));                // EN|ONESHOT|MODE=instr|CLEAR
    idle(3);                                                      // no retirement -> no progress
    expect_status(1, 0, 0, 0, 0, "D5 idle cycles do not advance instruction interval");
    for (int c = 0; c < 3; c++) begin                             // 2+2+2 = 6 >= 5 on third cycle
      drive(0, ALU, ADD, 5, 0, 5, 64'h100);
      drive(1, ALU, ADD, 5, 0, 5, 64'h104);
      step();
    end
    expect_status(2, 1, 0, 0, 1, "D5 interval ended on the 3rd dual-retire cycle");
    expect_word(0, 3, "D5 x5 count (both ports touch x5 every cycle: ONE count per cycle, 3 cycles)");
    expect_word(7, 6, "D5 retired = 6 (overshoot of 1)");

    // ------------------------------------------------------------------
    // D6: SNAPSHOT ends the interval now, also while disabled (IDLE)
    // ------------------------------------------------------------------
    $display("D6: snapshot");
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 0, 1, 0));                // CLEAR
    csr_wr(CSR_DFD_INTVL, 64'd1000);
    csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 0, 0, 0));
    drive(0, ALU, ADD, 5, 0, 5, 64'h200);                         step();
    drive(0, ALU, ADD, 5, 0, 0, 64'h204);                         step();
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 0, 0, 0));                // stop, live holds 2
    drive(0, ALU, ADD, 5, 0, 5, 64'h208);                         step();   // not counted
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 1, 0, 0));                // SNAPSHOT in IDLE
    expect_status(0, 1, 0, 0, 1, "D6 snapshot from IDLE: seq=1, NEW");
    expect_word(0, 2, "D6 snapshot captured the 2 counted accesses");

    // ------------------------------------------------------------------
    // D7: x0 can never be counted, even if configured sensitive
    // ------------------------------------------------------------------
    $display("D7: x0");
    csr_wr(CSR_DFD_SENS, sens_word(1, 0, 0, 0, 0, 0, 0, 0));
    csr_wr(CSR_DFD_CTRL, ctrl(1, 1, 0, 0, 0, 0));
    csr_wr(CSR_DFD_INTVL, 64'd3);
    drive(0, ALU, ADD, 0, 0, 0, 64'h300); step();
    drive(0, ALU, ADD, 0, 0, 0, 64'h304); step();
    drive(0, ALU, ADD, 0, 0, 0, 64'h308); step();
    expect_word(0, 0, "D7 x0 slot stays 0");
    expect_word(4 + 0, 64'h0, "D7 bitmap has no x0");

    // ------------------------------------------------------------------
    // D8: writing SENS discards data collected under the old table
    // ------------------------------------------------------------------
    $display("D8: SENS write clears");
    expect_status(2, 1, 0, 0, 1, "D8 before: DONE with data");
    csr_wr(CSR_DFD_SENS, sens_word(1, 7, 0, 0, 0, 0, 0, 0));
    expect_status(1, 0, 0, 0, 0, "D8 after SENS write: seq=0, NEW=0");
    expect_word(0, 0, "D8 shadow cleared");

    // ------------------------------------------------------------------
    // D9: saturation at 2^CntW-1, SAT flag, retire counter saturates too
    // ------------------------------------------------------------------
    $display("D9: saturation (%0d cycles)", MAXC + 5);
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 0, 1, 0));                // CLEAR
    csr_wr(CSR_DFD_SENS, sens_word(1, 9, 0, 0, 0, 0, 0, 0));
    csr_wr(CSR_DFD_INTVL, 64'(MAXC + 5));
    csr_wr(CSR_DFD_CTRL, ctrl(1, 1, 0, 0, 0, 0));
    for (int c = 0; c < MAXC + 5; c++) begin
      drive(0, ALU, ADD, 9, 0, 0, 64'h400);
      drive(1, ALU, ADD, 9, 0, 0, 64'h404);
      step();
    end
    expect_status(2, 1, 0, 1, 1, "D9 DONE, SAT set");
    expect_word(0, 64'(MAXC), "D9 x9 counter saturated at 2^CntW-1 (one count per cycle)");
    expect_word(7, 64'(MAXC), "D9 retire counter saturated (2 per cycle)");

    // ------------------------------------------------------------------
    // D10: CLEAR beats a same-cycle SNAPSHOT
    // ------------------------------------------------------------------
    $display("D10: clear vs snapshot");
    csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 1, 1, 0));
    expect_status(1, 0, 0, 0, 0, "D10 clear wins: seq=0, no NEW");
    expect_word(0, 0, "D10 shadow zero");

    // ------------------------------------------------------------------
    // D11: interval of 1 rolls every cycle
    // ------------------------------------------------------------------
    $display("D11: interval=1");
    csr_wr(CSR_DFD_INTVL, 64'd1);
    csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 0, 1, 0));
    drive(0, ALU, ADD, 9, 0, 0, 64'h500);                         step();
    expect_word(0, 0, "D11 last cycle (the SEL write cycle) had no events");

    // ------------------------------------------------------------------
    // R1: constrained-random regression vs reference model
    // ------------------------------------------------------------------
    $display("R1: random regression (%0d cycles)", 60000);
    csr_wr(CSR_DFD_CTRL, ctrl(0, 0, 0, 0, 1, 0));
    for (int c = 0; c < 60000; c++) begin
      rnd_commit();
      if ($urandom_range(0, 5) == 0) rnd_csr();
      step();
    end

    // ------------------------------------------------------------------
    // R2: long intervals, quiet CSR traffic (only SEL moves), hot registers
    //     -> counters saturate under random traffic; compared with the model
    // ------------------------------------------------------------------
    begin
      int sat_before;
      $display("R2: long-interval saturation under random traffic (%0d cycles)", 72000);
      csr_wr(CSR_DFD_SENS, sens_word(1, 1, 1, 2, 1, 3, 1, 4));
      csr_wr(CSR_DFD_INTVL, 64'd70000);
      csr_wr(CSR_DFD_CTRL, ctrl(1, 0, 0, 0, 1, 0));
      sat_before = cv_sat;
      for (int c = 0; c < 72000; c++) begin
        if ($urandom_range(0, 19) != 0) begin       // 95%: x1,x2,x3 via port 0, x4 via port 1
          drive(0, ALU, ADD, 1, 2, 3, 64'($urandom()));
          drive(1, ALU, ADD, 4, 0, 0, 64'($urandom()));
          maddr = 56'($urandom());
        end else rnd_commit();
        if ($urandom_range(0, 3) == 0) begin
          caddr = CSR_DFD_SEL; cwd = 64'($urandom_range(0, 12)); cwe = 1'b1;
        end
        step();
      end
      if (cv_sat == sat_before) fail("coverage: R2 never saturated");
    end

    $display("coverage: rolls=%0d overruns=%0d done=%0d sat=%0d snap_idle=%0d clears=%0d instr_mode=%0d dual_same_reg=%0d acks=%0d",
             cv_roll, cv_ovr, cv_done, cv_sat, cv_snap_idle, cv_clear, cv_instr_mode, cv_dual_same, cv_ack);
    if (cv_roll < 500)      fail("coverage: too few rolls");
    if (cv_ovr == 0)        fail("coverage: OVERRUN never exercised");
    if (cv_done == 0)       fail("coverage: one-shot DONE never exercised");
    if (cv_sat == 0)        fail("coverage: saturation never exercised");
    if (cv_snap_idle == 0)  fail("coverage: snapshot-in-IDLE never exercised");
    if (cv_clear == 0)      fail("coverage: CLEAR never exercised");
    if (cv_instr_mode == 0) fail("coverage: instruction mode never exercised");
    if (cv_dual_same == 0)  fail("coverage: dual-port same-register never exercised");

    if (errs == 0) $display("ALL DFDTMH TESTS PASSED");
    else           $display("DFDTMH TESTS FAILED: %0d error(s)", errs);
    $finish;
  end

  initial begin
    #(200_000_000);
    $display("TIMEOUT");
    $finish;
  end
endmodule