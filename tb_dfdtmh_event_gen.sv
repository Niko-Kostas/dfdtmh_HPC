// ------------------------------------------------------------------------------
// tb_dfdtmh_event_gen.sv -- self-checking testbench for dfdtmh_event_gen
//
// The DUT is purely combinational: drive inputs, wait, compare.
//
// Layer 1 (ED1..ED14): directed tests, expected values HAND-COMPUTED (no model).
// Layer 2 (ER1)      : exhaustive over (fu, op, use_zimm) x many register triples,
//                      single port, vs an independent reference function.
// Layer 3 (ER2)      : random dual-port streams vs the same reference.
//
// The reference deliberately uses different formulations from the RTL: block index
// by (addr >> shift) % blocks instead of bit-slicing, register touches via a
// per-instruction "touched" array, counts via arithmetic instead of increments.
// ------------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_event_gen;
  import ariane_pkg::*;
  import dfdtmh_pkg::*;

  localparam int unsigned NP = 2;
  localparam int unsigned RW = 2;   // retire_cnt_o width for NP=2

  typedef struct packed {
    logic [63:0] pc;
    fu_t         fu;
    fu_op        op;
    logic [4:0]  rs1, rs2, rd;
    logic        use_zimm;
  } e_t;

  e_t   [NP-1:0]      ci;
  logic [NP-1:0]      ack;
  logic [55:0]        maddr;
  logic [RegNum-1:0]  reg_inc;
  logic               pc_set, mem_set, retire;
  logic [PcBlkW-1:0]  pc_blk;
  logic [MemBlkW-1:0] mem_blk;
  logic [RW-1:0]      retire_cnt;

  dfdtmh_event_gen #(
      .NR_COMMIT_PORTS(NP), .scoreboard_entry_t(e_t), .MemAddrWidth(56), .RetW(RW)
  ) dut (
      .commit_instr_i(ci), .commit_ack_i(ack), .mem_addr_i(maddr),
      .reg_inc_o(reg_inc), .pc_set_o(pc_set), .pc_blk_o(pc_blk),
      .mem_set_o(mem_set), .mem_blk_o(mem_blk), .retire_o(retire),
      .retire_cnt_o(retire_cnt)
  );

  int errs = 0;
  int checks = 0;
  // coverage
  int cv_fu [fu_t];
  int cv_fp_ld, cv_fp_st, cv_zimm, cv_dual_same, cv_dual_br, cv_x0;

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------
  task automatic clear_in();
    ci = '0; ack = '0; maddr = '0;
  endtask

  task automatic drive(input int p, input fu_t fu, input fu_op op,
                       input int rs1, input int rs2, input int rd,
                       input logic [63:0] pc, input bit zimm = 1'b0);
    ci[p].fu = fu; ci[p].op = op;
    ci[p].rs1 = 5'(rs1); ci[p].rs2 = 5'(rs2); ci[p].rd = 5'(rd);
    ci[p].pc = pc; ci[p].use_zimm = zimm;
    ack[p] = 1'b1;
  endtask

  // compare against explicit expectations
  task automatic check(input logic [RegNum-1:0] e_reg, input bit e_pcs, input logic [PcBlkW-1:0] e_pcb,
                       input bit e_ms, input logic [MemBlkW-1:0] e_mb, input int e_cnt,
                       input string what);
    #1;
    checks++;
    if (reg_inc !== e_reg)
      begin errs++; $display("FAIL [%s] reg_inc_o = 0x%08h expected 0x%08h", what, reg_inc, e_reg); end
    if (pc_set !== e_pcs)
      begin errs++; $display("FAIL [%s] pc_set_o = %b expected %b", what, pc_set, e_pcs); end
    if (pc_blk !== e_pcb)
      begin errs++; $display("FAIL [%s] pc_blk_o = 0x%04h expected 0x%04h", what, pc_blk, e_pcb); end
    if (mem_set !== e_ms)
      begin errs++; $display("FAIL [%s] mem_set_o = %b expected %b", what, mem_set, e_ms); end
    if (mem_blk !== e_mb)
      begin errs++; $display("FAIL [%s] mem_blk_o = 0x%04h expected 0x%04h", what, mem_blk, e_mb); end
    if (retire_cnt !== RW'(e_cnt))
      begin errs++; $display("FAIL [%s] retire_cnt_o = %0d expected %0d", what, retire_cnt, e_cnt); end
    if (retire !== (e_cnt != 0))
      begin errs++; $display("FAIL [%s] retire_o = %b expected %b", what, retire, (e_cnt != 0)); end
  endtask

  function automatic logic [RegNum-1:0] regs(input int a = -1, input int b = -1, input int c = -1,
                                             input int d = -1, input int e = -1, input int f = -1);
    logic [RegNum-1:0] m;
    m = '0;
    if (a >= 0) m[a] = 1'b1;
    if (b >= 0) m[b] = 1'b1;
    if (c >= 0) m[c] = 1'b1;
    if (d >= 0) m[d] = 1'b1;
    if (e >= 0) m[e] = 1'b1;
    if (f >= 0) m[f] = 1'b1;
    return m;
  endfunction

  // ------------------------------------------------------------------
  // independent reference
  // ------------------------------------------------------------------
  typedef struct {
    bit [RegNum-1:0]  reg_inc;
    bit               pc_set;
    bit [PcBlkW-1:0]  pc_blk;
    bit               mem_set;
    bit [MemBlkW-1:0] mem_blk;
    int               cnt;
  } exp_t;

  function automatic exp_t ref_model();
    exp_t x;
    x.reg_inc = '0; x.pc_set = 0; x.pc_blk = '0; x.mem_set = 0; x.mem_blk = '0; x.cnt = 0;
    for (int p = 0; p < NP; p++) begin
      if (ack[p]) begin
        bit touched [RegNum];
        bit int_unit, rs1_ok, rs2_ok, rd_ok;
        int blk;
        x.cnt = x.cnt + 1;
        for (int r = 0; r < RegNum; r++) touched[r] = 0;
        // which units use integer registers
        int_unit = 0;
        if (ci[p].fu == ALU || ci[p].fu == MULT || ci[p].fu == CTRL_FLOW ||
            ci[p].fu == LOAD || ci[p].fu == STORE || ci[p].fu == CSR) int_unit = 1;
        rs1_ok = int_unit;
        rs2_ok = int_unit;
        rd_ok  = int_unit;
        if (ci[p].fu == CSR && ci[p].use_zimm) rs1_ok = 0;
        if (ci[p].fu == STORE && (ci[p].op == FSB || ci[p].op == FSH || ci[p].op == FSW || ci[p].op == FSD)) rs2_ok = 0;
        if (ci[p].fu == LOAD  && (ci[p].op == FLB || ci[p].op == FLH || ci[p].op == FLW || ci[p].op == FLD)) rd_ok  = 0;
        if (rs1_ok && ci[p].rs1 != 0) touched[ci[p].rs1] = 1;
        if (rs2_ok && ci[p].rs2 != 0) touched[ci[p].rs2] = 1;
        if (rd_ok  && ci[p].rd  != 0) touched[ci[p].rd]  = 1;
        for (int r = 0; r < RegNum; r++) if (touched[r]) x.reg_inc[r] = 1;
        if (ci[p].fu == CTRL_FLOW) begin
          blk = int'((ci[p].pc >> PcShift) % PcBlkW);
          x.pc_set = 1;
          x.pc_blk[blk] = 1;
        end
        if (ci[p].fu == LOAD || ci[p].fu == STORE) x.mem_set = 1;
      end
    end
    if (x.mem_set) begin
      int mblk;
      mblk = int'((maddr >> MemShift) % MemBlkW);
      x.mem_blk[mblk] = 1;
    end
    return x;
  endfunction

  task automatic check_vs_ref(input string what);
    exp_t x;
    x = ref_model();
    check(x.reg_inc, x.pc_set, x.pc_blk, x.mem_set, x.mem_blk, x.cnt, what);
  endtask

  function automatic fu_t pick_fu();
    case ($urandom_range(0, 7))
      0: return ALU;  1: return CTRL_FLOW; 2: return MULT; 3: return CSR;
      4: return LOAD; 5: return STORE;     6: return FPU;  default: return ACCEL;
    endcase
  endfunction

  function automatic fu_op pick_op();
    fu_op o;
    case ($urandom_range(0, 18))
      0: o = ADD;   1: o = SUB;  2: o = XORL; 3: o = ORL;  4: o = ANDL; 5: o = MUL;
      6: o = BRANCH; 7: o = LD;  8: o = SD;   9: o = FLD; 10: o = FLW; 11: o = FLH;
      12: o = FLB;  13: o = FSD; 14: o = FSW; 15: o = FSH; 16: o = FSB;
      17: o = CSR_WRITE; default: o = CSR_READ;
    endcase
    return o;
  endfunction

  // ------------------------------------------------------------------
  // main
  // ------------------------------------------------------------------
  initial begin
    fu_t all_fu [8];
    fu_op all_op [19];
    cv_fp_ld = 0; cv_fp_st = 0; cv_zimm = 0; cv_dual_same = 0; cv_dual_br = 0; cv_x0 = 0;
    all_fu = '{ALU, CTRL_FLOW, MULT, CSR, LOAD, STORE, FPU, ACCEL};
    all_op = '{ADD, SUB, XORL, ORL, ANDL, MUL, BRANCH, LD, SD, FLD, FLW, FLH, FLB,
               FSD, FSW, FSH, FSB, CSR_WRITE, CSR_READ};

    // ---- ED1: nothing retires ------------------------------------------------
    $display("ED1: no ack -> all outputs zero");
    clear_in();
    ci[0].fu = ALU; ci[0].rs1 = 5'd3; ci[0].rs2 = 5'd4; ci[0].rd = 5'd5;   // present but not acked
    ci[1].fu = LOAD; ci[1].rs1 = 5'd6;
    maddr = 56'hFFFF_FFFF;
    check('0, 0, '0, 0, '0, 0, "ED1");

    // ---- ED2: plain ALU -------------------------------------------------------
    $display("ED2: ALU rs1/rs2/rd");
    clear_in();
    drive(0, ALU, ADD, 5, 6, 7, 64'h100);
    check(regs(5, 6, 7), 0, '0, 0, '0, 1, "ED2 ALU touches x5,x6,x7, no pc/mem");

    // ---- ED3: x0 ignored --------------------------------------------------------
    $display("ED3: x0 never touched");
    clear_in();
    drive(0, ALU, ADD, 0, 0, 0, 64'h100);
    check('0, 0, '0, 0, '0, 1, "ED3 all-x0 ALU retires but touches nothing");
    clear_in();
    drive(0, ALU, ADD, 0, 9, 0, 64'h100);
    check(regs(9), 0, '0, 0, '0, 1, "ED3b only x9");

    // ---- ED4: MULT --------------------------------------------------------------
    $display("ED4: MULT");
    clear_in();
    drive(0, MULT, MUL, 10, 11, 12, 64'h100);
    check(regs(10, 11, 12), 0, '0, 0, '0, 1, "ED4");

    // ---- ED5: integer vs FP loads ---------------------------------------------
    $display("ED5: loads (maddr 0x1230 -> block (0x1230>>4) mod 16 = 3)");
    clear_in(); maddr = 56'h1230;
    drive(0, LOAD, LD, 10, 0, 11, 64'h100);
    check(regs(10, 11), 0, '0, 1, MemBlkW'(1) << 3, 1, "ED5 integer load: rs1 and rd counted");
    foreach (all_op[i]) begin
      if (all_op[i] inside {FLB, FLH, FLW, FLD}) begin
        clear_in(); maddr = 56'h1230;
        drive(0, LOAD, all_op[i], 10, 0, 11, 64'h100);
        check(regs(10), 0, '0, 1, MemBlkW'(1) << 3, 1, $sformatf("ED5 %s: rd is FP -> masked, rs1 base kept", all_op[i].name()));
      end
    end

    // ---- ED6: integer vs FP stores --------------------------------------------
    $display("ED6: stores (maddr 0x1250 -> block 5)");
    clear_in(); maddr = 56'h1250;
    drive(0, STORE, SD, 13, 10, 0, 64'h100);
    check(regs(13, 10), 0, '0, 1, MemBlkW'(1) << 5, 1, "ED6 integer store: rs1 and rs2 counted");
    foreach (all_op[i]) begin
      if (all_op[i] inside {FSB, FSH, FSW, FSD}) begin
        clear_in(); maddr = 56'h1250;
        drive(0, STORE, all_op[i], 13, 10, 0, 64'h100);
        check(regs(13), 0, '0, 1, MemBlkW'(1) << 5, 1, $sformatf("ED6 %s: rs2 is FP -> masked, rs1 base kept", all_op[i].name()));
      end
    end
    // an integer store whose rd field is non-zero must still count rd? (AMO-style writes rd)
    clear_in(); maddr = 56'h1250;
    drive(0, STORE, SD, 13, 10, 14, 64'h100);
    check(regs(13, 10, 14), 0, '0, 1, MemBlkW'(1) << 5, 1, "ED6 AMO-style store: rd counted");

    // ---- ED7: CSR immediate forms ------------------------------------------------
    $display("ED7: CSR");
    clear_in();
    drive(0, CSR, CSR_WRITE, 10, 0, 14, 64'h100, 1'b1);
    check(regs(14), 0, '0, 0, '0, 1, "ED7 CSRRWI: rs1 is an immediate -> masked, rd counted");
    clear_in();
    drive(0, CSR, CSR_READ, 10, 0, 15, 64'h100, 1'b0);
    check(regs(10, 15), 0, '0, 0, '0, 1, "ED7 CSRRW: rs1 and rd counted");

    // ---- ED8: FPU / ACCEL ignored -------------------------------------------------
    $display("ED8: FPU / ACCEL");
    clear_in();
    drive(0, FPU, ADD, 5, 6, 7, 64'h100);
    check('0, 0, '0, 0, '0, 1, "ED8 FPU instruction: FP regs not counted, still retires");
    clear_in();
    drive(0, ACCEL, ADD, 5, 6, 7, 64'h100);
    check('0, 0, '0, 0, '0, 1, "ED8 ACCEL instruction ignored, still retires");

    // ---- ED9: branches / PC blocks (PcShift=2, 16 blocks -> block = pc[5:2]) -------
    $display("ED9: branch PC blocks");
    clear_in();
    drive(0, CTRL_FLOW, BRANCH, 5, 7, 0, 64'h20);        // (0x20>>2)%16 = 8
    check(regs(5, 7), 1, PcBlkW'(1) << 8, 0, '0, 1, "ED9 branch pc=0x20 -> block 8");
    clear_in();
    drive(0, CTRL_FLOW, BRANCH, 0, 0, 1, 64'h3C);        // jal x1: (0x3C>>2)%16 = 15
    check(regs(1), 1, PcBlkW'(1) << 15, 0, '0, 1, "ED9 jal pc=0x3C -> block 15");
    clear_in();
    drive(0, CTRL_FLOW, BRANCH, 0, 0, 0, 64'h40);        // (0x40>>2)%16 = 0 (aliases block 0)
    check('0, 1, PcBlkW'(1) << 0, 0, '0, 1, "ED9 pc=0x40 aliases back to block 0");
    clear_in();
    drive(0, ALU, ADD, 1, 2, 3, 64'h20);                 // not a branch: no pc event
    check(regs(1, 2, 3), 0, '0, 0, '0, 1, "ED9 non-branch has no pc event");
    clear_in();
    drive(0, LOAD, LD, 1, 0, 2, 64'h20); maddr = 56'h0;
    check(regs(1, 2), 0, '0, 1, MemBlkW'(1), 1, "ED9 load has no pc event");

    // ---- ED10: memory blocks (MemShift=4, 16 blocks -> block = addr[7:4]) ----------
    $display("ED10: memory blocks");
    clear_in(); maddr = 56'h0;
    drive(0, STORE, SD, 1, 2, 0, 64'h100);
    check(regs(1, 2), 0, '0, 1, MemBlkW'(1) << 0, 1, "ED10 addr 0x0 -> block 0");
    clear_in(); maddr = 56'hF0;
    drive(0, LOAD, LD, 1, 0, 2, 64'h100);
    check(regs(1, 2), 0, '0, 1, MemBlkW'(1) << 15, 1, "ED10 addr 0xF0 -> block 15");
    clear_in(); maddr = 56'h10F;
    drive(0, LOAD, LD, 1, 0, 2, 64'h100);
    check(regs(1, 2), 0, '0, 1, MemBlkW'(1) << 0, 1, "ED10 addr 0x10F -> block (0x10F>>4)%16 = 0");
    clear_in(); maddr = 56'hDEAD_BEEF;                   // garbage address, no memory instruction
    drive(0, ALU, ADD, 1, 2, 3, 64'h100);
    check(regs(1, 2, 3), 0, '0, 0, '0, 1, "ED10 garbage mem_addr ignored when no load/store retires");
    clear_in(); maddr = 56'h70;                          // load present but NOT acked
    ci[0].fu = LOAD; ci[0].rs1 = 5'd4;
    check('0, 0, '0, 0, '0, 0, "ED10 un-acked load generates nothing");

    // ---- ED11: dual port merging --------------------------------------------------
    $display("ED11: dual port");
    clear_in();
    drive(1, ALU, ADD, 5, 0, 6, 64'h100);
    check(regs(5, 6), 0, '0, 0, '0, 1, "ED11 only port 1 acked");
    clear_in();
    drive(0, ALU, ADD, 1, 2, 3, 64'h100);
    drive(1, ALU, ADD, 4, 5, 6, 64'h104);
    check(regs(1, 2, 3, 4, 5, 6), 0, '0, 0, '0, 2, "ED11 two instructions, registers OR-merged, count=2");
    clear_in();
    drive(0, ALU, ADD, 5, 5, 5, 64'h100);
    drive(1, ALU, ADD, 5, 5, 5, 64'h104);
    check(regs(5), 0, '0, 0, '0, 2, "ED11 same register on both ports: ONE touch per cycle, count 2");
    clear_in();
    drive(0, CTRL_FLOW, BRANCH, 1, 2, 0, 64'h08);        // block 2
    drive(1, CTRL_FLOW, BRANCH, 3, 4, 0, 64'h14);        // block 5
    check(regs(1, 2, 3, 4), 1, (PcBlkW'(1) << 2) | (PcBlkW'(1) << 5), 0, '0, 2,
          "ED11 two branches retire together: BOTH pc blocks recorded");
    clear_in();
    drive(0, ALU, ADD, 1, 2, 3, 64'h100);
    drive(1, STORE, SD, 4, 5, 0, 64'h104); maddr = 56'h2A0;   // block (0x2A0>>4)%16 = 10
    check(regs(1, 2, 3, 4, 5), 0, '0, 1, MemBlkW'(1) << 10, 2, "ED11 mixed: mem op on port 1");
    clear_in();
    drive(0, LOAD, LD, 1, 0, 2, 64'h100); maddr = 56'h30;     // block 3
    drive(1, ALU, ADD, 3, 4, 5, 64'h104);
    check(regs(1, 2, 3, 4, 5), 0, '0, 1, MemBlkW'(1) << 3, 2, "ED11 mixed: mem op on port 0");
    // port 1 FP load next to port 0 integer op: masking is per port
    clear_in(); maddr = 56'h50;
    drive(0, ALU, ADD, 1, 2, 3, 64'h100);
    drive(1, LOAD, FLD, 4, 0, 3, 64'h104);
    check(regs(1, 2, 3, 4), 0, '0, 1, MemBlkW'(1) << 5, 2, "ED11 port1 FLD: rd masked but x3 still touched by port0");

    // ---- ED12: retire count ---------------------------------------------------------
    $display("ED12: retire count");
    clear_in(); check('0, 0, '0, 0, '0, 0, "ED12 none");
    clear_in(); drive(0, FPU, ADD, 0, 0, 0, 64'h0); check('0, 0, '0, 0, '0, 1, "ED12 one (FPU still retires)");
    clear_in(); drive(0, FPU, ADD, 0, 0, 0, 64'h0); drive(1, ACCEL, ADD, 0, 0, 0, 64'h4);
    check('0, 0, '0, 0, '0, 2, "ED12 two");

    // ---- ED13: upper PC / address bits don't leak into the block index --------------
    $display("ED13: high bits");
    clear_in();
    drive(0, CTRL_FLOW, BRANCH, 0, 0, 0, 64'hFFFF_FFFF_8000_0020);
    check('0, 1, PcBlkW'(1) << 8, 0, '0, 1, "ED13 high PC bits ignored (block still 8)");
    clear_in(); maddr = 56'hFF_0000_1230;
    drive(0, LOAD, LD, 0, 0, 0, 64'h0);
    check('0, 0, '0, 1, MemBlkW'(1) << 3, 1, "ED13 high address bits ignored (block still 3)");

    // ---- ED14: register extremes -----------------------------------------------------
    $display("ED14: x31");
    clear_in();
    drive(0, ALU, ADD, 31, 31, 31, 64'h0);
    check(regs(31), 0, '0, 0, '0, 1, "ED14 x31");
    clear_in();
    drive(0, ALU, ADD, 1, 31, 16, 64'h0);
    check(regs(1, 31, 16), 0, '0, 0, '0, 1, "ED14 x1,x31,x16");

    // ---- ER1: exhaustive over (fu, op, zimm), many register triples ------------------
    $display("ER1: exhaustive (fu x op x zimm) x 64 random register triples, port 0 and port 1");
    for (int f = 0; f < 8; f++) begin
      for (int o = 0; o < 19; o++) begin
        for (int z = 0; z < 2; z++) begin
          for (int t = 0; t < 64; t++) begin
            int port;
            port = t % 2;
            clear_in();
            maddr = 56'($urandom()) << 8 | 56'($urandom());
            drive(port, all_fu[f], all_op[o], $urandom_range(0, 31), $urandom_range(0, 31),
                  $urandom_range(0, 31), 64'($urandom()) << 32 | 64'($urandom()), 1'(z));
            check_vs_ref($sformatf("ER1 fu=%s op=%s zimm=%0d", all_fu[f].name(), all_op[o].name(), z));
          end
        end
      end
    end

    // ---- ER2: random dual-port streams -------------------------------------------------
    $display("ER2: random dual-port (100000 vectors)");
    for (int c = 0; c < 100000; c++) begin
      clear_in();
      maddr = 56'($urandom()) << 8 | 56'($urandom());
      for (int p = 0; p < NP; p++) begin
        // small register range makes same-register collisions between ports frequent
        if ($urandom_range(0, 9) < 7)
          drive(p, pick_fu(), pick_op(), $urandom_range(0, 7), $urandom_range(0, 7),
                $urandom_range(0, 7), 64'($urandom()) << 32 | 64'($urandom()), 1'($urandom_range(0, 1)));
      end
      check_vs_ref("ER2 random");
      // coverage bookkeeping
      if (ack == 2'b11 && (ci[0].rd == ci[1].rd) && ci[0].rd != 0) cv_dual_same++;
      if (ack == 2'b11 && ci[0].fu == CTRL_FLOW && ci[1].fu == CTRL_FLOW) cv_dual_br++;
      for (int p = 0; p < NP; p++) if (ack[p]) begin
        if (ci[p].fu == LOAD  && ci[p].op inside {FLB, FLH, FLW, FLD}) cv_fp_ld++;
        if (ci[p].fu == STORE && ci[p].op inside {FSB, FSH, FSW, FSD}) cv_fp_st++;
        if (ci[p].fu == CSR && ci[p].use_zimm) cv_zimm++;
        if (ci[p].rs1 == 0 || ci[p].rs2 == 0 || ci[p].rd == 0) cv_x0++;
        cv_fu[ci[p].fu] = cv_fu[ci[p].fu] + 1;
      end
    end
    $display("coverage: fp_ld=%0d fp_st=%0d csr_zimm=%0d dual_same_rd=%0d dual_branch=%0d x0=%0d fus=%0d",
             cv_fp_ld, cv_fp_st, cv_zimm, cv_dual_same, cv_dual_br, cv_x0, cv_fu.num());
    if (cv_fp_ld < 100 || cv_fp_st < 100 || cv_zimm < 100) begin errs++; $display("FAIL: coverage: masking cases rarely hit"); end
    if (cv_dual_same < 100) begin errs++; $display("FAIL: coverage: dual-port same-register rarely hit"); end
    if (cv_dual_br < 100)   begin errs++; $display("FAIL: coverage: dual-branch rarely hit"); end
    if (cv_fu.num() != 8)   begin errs++; $display("FAIL: coverage: not every functional unit was exercised"); end

    $display("checks run: %0d", checks);
    if (errs == 0) $display("ALL DFDTMH_EVENT_GEN TESTS PASSED");
    else           $display("DFDTMH_EVENT_GEN TESTS FAILED: %0d error(s)", errs);
    $finish;
  end
endmodule