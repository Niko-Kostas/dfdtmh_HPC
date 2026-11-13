// ------------------------------------------------------------------------------
// tb_dfdtmh_counter_bank.sv -- self-checking testbench for dfdtmh_counter_bank
//
// Layer 1 (CD1..CD12): directed tests with HAND-COMPUTED expectations.
// Layer 2 (CR1/CR2)  : random, compared EVERY CYCLE against a reference model
//                      (all NumWords readout words + the shadow SAT flag).
//
// Timing: inputs change right after a negedge; the DUT samples at the next posedge;
// outputs are compared one negedge later.
// ------------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_counter_bank;
  import dfdtmh_pkg::*;

  localparam int unsigned RW   = 2;
  localparam longint unsigned MAXC = (64'd1 << CntW) - 1;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  logic [RegNum-1:0]  reg_inc;
  logic [PcBlkW-1:0]  pc_blk;
  logic [MemBlkW-1:0] mem_blk;
  logic [RW-1:0]      ret_cnt;
  sens_table_t        sens;
  logic               acc_en, roll, clear;
  logic [31:0]        word [NumWords];
  logic               sat;

  dfdtmh_counter_bank #(.RetW(RW)) dut (
      .clk_i(clk), .rst_ni(rst_n),
      .reg_inc_i(reg_inc), .pc_blk_i(pc_blk), .mem_blk_i(mem_blk), .retire_cnt_i(ret_cnt),
      .sens_i(sens), .acc_en_i(acc_en), .roll_i(roll), .clear_i(clear),
      .word_o(word), .shadow_sat_o(sat)
  );

  // ======================================================================
  // Reference model
  // ======================================================================
  longint unsigned l_cnt [NrSens];  bit [31:0] l_reg;  bit [31:0] l_pc;  bit [31:0] l_mem;
  longint unsigned l_ret;           bit l_sat;
  longint unsigned s_cnt [NrSens];  bit [31:0] s_reg;  bit [31:0] s_pc;  bit [31:0] s_mem;
  longint unsigned s_ret;           bit s_sat;

  int errs = 0;
  int checks = 0;
  int cv_roll, cv_clear, cv_sat, cv_roll_noacc, cv_dupslot, cv_invalid_hit;

  function automatic bit ref_is_sens(int r);
    for (int s = 0; s < NrSens; s++) if (sens[s].valid && int'(sens[s].reg_idx) == r) return 1'b1;
    return 1'b0;
  endfunction

  task automatic ref_reset();
    for (int s = 0; s < NrSens; s++) begin l_cnt[s] = 0; s_cnt[s] = 0; end
    l_reg = 0; l_pc = 0; l_mem = 0; l_ret = 0; l_sat = 0;
    s_reg = 0; s_pc = 0; s_mem = 0; s_ret = 0; s_sat = 0;
  endtask

  task automatic ref_step();
    // accumulate
    if (acc_en) begin
      for (int s = 0; s < NrSens; s++) begin
        if (sens[s].valid && reg_inc[sens[s].reg_idx]) begin
          if (l_cnt[s] == MAXC) l_sat = 1; else l_cnt[s] = l_cnt[s] + 1;
        end
        if (sens[s].valid == 0 && reg_inc[sens[s].reg_idx]) cv_invalid_hit++;
      end
      for (int r = 0; r < RegNum; r++) if (reg_inc[r] && !ref_is_sens(r)) l_reg[r] = 1'b1;
      l_pc  |= 32'(pc_blk);
      l_mem |= 32'(mem_blk);
      l_ret = l_ret + ret_cnt;
      if (l_ret > MAXC) begin l_ret = MAXC; l_sat = 1; end
    end
    // clear beats roll
    if (clear) begin
      for (int s = 0; s < NrSens; s++) begin l_cnt[s] = 0; s_cnt[s] = 0; end
      l_reg = 0; l_pc = 0; l_mem = 0; l_ret = 0; l_sat = 0;
      s_reg = 0; s_pc = 0; s_mem = 0; s_ret = 0; s_sat = 0;
      cv_clear++;
    end else if (roll) begin
      for (int s = 0; s < NrSens; s++) begin s_cnt[s] = l_cnt[s]; l_cnt[s] = 0; end
      s_reg = l_reg; l_reg = 0;
      s_pc  = l_pc;  l_pc  = 0;
      s_mem = l_mem; l_mem = 0;
      s_ret = l_ret; l_ret = 0;
      s_sat = l_sat; l_sat = 0;
      cv_roll++;
      if (!acc_en) cv_roll_noacc++;
      if (s_sat) cv_sat++;
    end
  endtask

  function automatic bit [31:0] exp_word(int w);
    if (w < NrSens)           return 32'(s_cnt[w]);
    else if (w == NrSens)     return s_reg;
    else if (w == NrSens + 1) return s_pc;
    else if (w == NrSens + 2) return s_mem;
    else                      return 32'(s_ret);
  endfunction

  task automatic compare(input string tag);
    for (int w = 0; w < NumWords; w++) begin
      checks++;
      if (word[w] !== exp_word(w)) begin
        errs++;
        $display("[%0t] FAIL %s: word[%0d] = 0x%0h expected 0x%0h", $time, tag, w, word[w], exp_word(w));
        if (errs >= 25) begin $display("too many errors"); $finish; end
      end
    end
    checks++;
    if (sat !== s_sat) begin
      errs++;
      $display("[%0t] FAIL %s: shadow_sat = %b expected %b", $time, tag, sat, s_sat);
    end
  endtask

  // one clock: reference first, DUT samples, compare, drop to idle inputs
  task automatic step();
    ref_step();
    @(posedge clk);
    @(negedge clk);
    compare("cycle");
    reg_inc = '0; pc_blk = '0; mem_blk = '0; ret_cnt = '0;
    acc_en = 1'b1; roll = 1'b0; clear = 1'b0;
  endtask

  // ---------- stimulus helpers -------------------------------------------------
  function automatic logic [RegNum-1:0] regs(input int a = -1, input int b = -1, input int c = -1, input int d = -1);
    logic [RegNum-1:0] m;
    m = '0;
    if (a >= 0) m[a] = 1'b1;
    if (b >= 0) m[b] = 1'b1;
    if (c >= 0) m[c] = 1'b1;
    if (d >= 0) m[d] = 1'b1;
    return m;
  endfunction

  task automatic set_sens(input int s, input bit v, input int idx);
    sens[s].valid = v;
    sens[s].reg_idx = 5'(idx);
  endtask

  task automatic ev(input logic [RegNum-1:0] r, input int pb, input int mb, input int rc);
    reg_inc = r;
    pc_blk  = (pb >= 0) ? (PcBlkW'(1) << pb)  : '0;
    mem_blk = (mb >= 0) ? (MemBlkW'(1) << mb) : '0;
    ret_cnt = RW'(rc);
  endtask

  // absolute expectations (independent of the reference)
  task automatic expect_w(input int w, input longint unsigned e, input string what);
    checks++;
    if (word[w] !== 32'(e)) begin
      errs++;
      $display("[%0t] FAIL %s: word[%0d] = 0x%0h expected 0x%0h", $time, what, w, word[w], e);
    end
  endtask

  task automatic expect_all_zero(input string what);
    for (int w = 0; w < NumWords; w++) expect_w(w, 0, what);
    checks++;
    if (sat !== 1'b0) begin errs++; $display("FAIL %s: shadow_sat should be 0", what); end
  endtask

  task automatic default_sens();
    sens = '0;
    set_sens(0, 1, 5);
    set_sens(1, 1, 10);
    set_sens(2, 0, 7);     // INVALID slot pointing at x7: must be ignored
    set_sens(3, 0, 0);
  endtask

  task automatic do_clear();
    clear = 1'b1; step();
  endtask

  // ======================================================================
  // main
  // ======================================================================
  initial begin
    rst_n = 1'b0; reg_inc = '0; pc_blk = '0; mem_blk = '0; ret_cnt = '0;
    acc_en = 1'b1; roll = 1'b0; clear = 1'b0; sens = '0;
    cv_roll = 0; cv_clear = 0; cv_sat = 0; cv_roll_noacc = 0; cv_dupslot = 0; cv_invalid_hit = 0;
    ref_reset();
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- CD1: reset --------------------------------------------------------------
    $display("CD1: reset");
    expect_all_zero("CD1");

    // ---- CD2: hand-computed interval -----------------------------------------------
    //   sensitive: slot0=x5, slot1=x10, slot2 INVALID(x7), slot3 INVALID(x0)
    $display("CD2: hand-computed interval");
    default_sens();
    ev(regs(5, 6),     2, -1, 1); step();                       // c0
    ev(regs(5, 10, 7), -1, 9, 2); step();                       // c1  (x7 touched, slot2 invalid)
    ev(regs(10),       -1, -1, 1); step();                      // c2
    ev(regs(5),        -1, -1, 1); roll = 1'b1; step();         // c3  roll cycle: its events count
    expect_w(0, 3,      "CD2 x5 count (c0,c1,c3)");
    expect_w(1, 2,      "CD2 x10 count (c1,c2)");
    expect_w(2, 0,      "CD2 invalid slot2 stays 0 even though x7 was touched");
    expect_w(3, 0,      "CD2 invalid slot3 stays 0");
    expect_w(NrSens,     64'h00C0, "CD2 reg bitmap = x6,x7 only (x5,x10 are sensitive)");
    expect_w(NrSens + 1, 64'h0004, "CD2 pc bitmap = block 2");
    expect_w(NrSens + 2, 64'h0200, "CD2 mem bitmap = block 9");
    expect_w(NrSens + 3, 5,        "CD2 retire count 1+2+1+1");
    checks++; if (sat) begin errs++; $display("FAIL CD2: sat must be 0"); end

    // ---- CD3: roll zeroes live, next cycle already belongs to the new interval ------
    $display("CD3: no dead cycle after roll");
    ev(regs(5), 3, 4, 1); step();                               // first cycle after the roll
    ev('0, -1, -1, 0); roll = 1'b1; step();                     // end the new interval, no events
    expect_w(0, 1,      "CD3 x5: event right after roll is counted in the NEW interval");
    expect_w(1, 0,      "CD3 x10 zero: old interval's counts did not leak");
    expect_w(NrSens,     64'h0, "CD3 reg bitmap empty (x5 sensitive)");
    expect_w(NrSens + 1, 64'h0008, "CD3 pc block 3");
    expect_w(NrSens + 2, 64'h0010, "CD3 mem block 4");
    expect_w(NrSens + 3, 1,        "CD3 retire 1");
    ev('0, -1, -1, 0); roll = 1'b1; step();                     // empty interval
    expect_all_zero("CD3 empty interval after roll (live was zeroed)");

    // ---- CD4: acc_en=0 holds, roll without acc_en snapshots partial data ------------
    $display("CD4: acc_en gating / snapshot");
    ev(regs(5, 20), 1, 1, 1); step();
    ev(regs(5, 21), 1, 1, 1); step();
    for (int i = 0; i < 3; i++) begin
      ev(regs(5, 22, 10), 7, 7, 2); acc_en = 1'b0; step();     // ignored
    end
    ev(regs(5, 23), 8, 8, 2); acc_en = 1'b0; roll = 1'b1; step();   // snapshot, still ignores this cycle's events
    expect_w(0, 2,      "CD4 x5: 2 accumulated cycles only");
    expect_w(1, 0,      "CD4 x10: only touched while acc_en=0");
    expect_w(NrSens,     (64'd1 << 20) | (64'd1 << 21), "CD4 bitmap: x20,x21 only");
    expect_w(NrSens + 1, 64'h0002, "CD4 pc block 1 only");
    expect_w(NrSens + 2, 64'h0002, "CD4 mem block 1 only");
    expect_w(NrSens + 3, 2,        "CD4 retire 2");

    // ---- CD5: bitmaps are sticky ORs over the interval ------------------------------
    $display("CD5: sticky bitmaps");
    ev(regs(12), 0, 0, 1);  step();
    ev(regs(13), 15, 15, 1); step();
    ev(regs(12), 0, 0, 1);  step();
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(NrSens,     (64'd1 << 12) | (64'd1 << 13), "CD5 reg bitmap");
    expect_w(NrSens + 1, 64'h8001, "CD5 pc blocks 0 and 15");
    expect_w(NrSens + 2, 64'h8001, "CD5 mem blocks 0 and 15");
    expect_w(NrSens + 3, 3,        "CD5 retire 3");

    // ---- CD6: CLEAR beats ROLL, clears shadow AND live --------------------------------
    $display("CD6: clear");
    ev(regs(5, 30), 5, 5, 2); step();
    ev(regs(5, 31), 6, 6, 2); roll = 1'b1; step();              // shadow now has data
    expect_w(0, 2, "CD6 precondition: shadow holds x5=2");
    ev(regs(5, 8), 9, 9, 2); step();                            // live has data too
    ev(regs(5), 3, 3, 1); clear = 1'b1; roll = 1'b1; step();    // clear + roll same cycle
    expect_all_zero("CD6 clear+roll: shadow zero (clear wins)");
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_all_zero("CD6 live was cleared too (next roll captures nothing)");

    // ---- CD7: duplicate slots both count; table reprogramming takes effect -------------
    $display("CD7: duplicate slots / new table");
    sens = '0;
    set_sens(0, 1, 3);
    set_sens(1, 1, 3);
    set_sens(2, 1, 4);
    set_sens(3, 0, 3);
    do_clear();
    ev(regs(3), -1, -1, 1); step();
    ev(regs(3, 4), -1, -1, 1); step();
    ev(regs(4), -1, -1, 1); roll = 1'b1; step();
    expect_w(0, 2, "CD7 slot0 x3");
    expect_w(1, 2, "CD7 slot1 (duplicate of x3) counts too");
    expect_w(2, 2, "CD7 slot2 x4");
    expect_w(3, 0, "CD7 slot3 invalid");
    expect_w(NrSens, 64'h0, "CD7 reg bitmap empty: x3 and x4 are sensitive");
    // make x3 non-sensitive: its touches now go to the bitmap
    set_sens(0, 0, 3); set_sens(1, 0, 3);
    ev(regs(3, 4), -1, -1, 0); step();
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(0, 0, "CD7b slots invalid -> 0");
    expect_w(2, 1, "CD7b slot2 still counts x4");
    expect_w(NrSens, 64'h8, "CD7b x3 now in the bitmap");

    // ---- CD8: counter saturation ---------------------------------------------------------
    $display("CD8: counter saturation (%0d cycles)", MAXC + 6);
    sens = '0; set_sens(0, 1, 9);
    do_clear();
    for (longint unsigned c = 0; c < MAXC + 5; c++) begin ev(regs(9), -1, -1, 0); step(); end
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(0, MAXC, "CD8 x9 saturated at 2^CntW-1");
    checks++; if (!sat) begin errs++; $display("FAIL CD8: SAT flag must be set"); end
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_all_zero("CD8 next interval: counter and SAT flag are clean again");
    // exactly at the limit: no saturation flag
    do_clear();
    for (longint unsigned c = 0; c < MAXC; c++) begin ev(regs(9), -1, -1, 0); step(); end
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(0, MAXC, "CD8 exactly 2^CntW-1 events: value");
    checks++; if (sat) begin errs++; $display("FAIL CD8: SAT must NOT be set when the count just fits"); end

    // ---- CD9: retire-count saturation sets SAT independently of the counters --------------
    $display("CD9: retire count saturation");
    sens = '0; set_sens(0, 1, 9);
    do_clear();
    for (longint unsigned c = 0; c < (MAXC / 2) + 3; c++) begin ev('0, -1, -1, 2); step(); end
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(NrSens + 3, MAXC, "CD9 retire counter saturated");
    expect_w(0, 0,             "CD9 sensitive counter untouched");
    checks++; if (!sat) begin errs++; $display("FAIL CD9: SAT flag must be set by the retire counter"); end

    // ---- CD10: roll with nothing to roll after reset-like state ----------------------------
    $display("CD10: first-interval roll");
    do_clear();
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_all_zero("CD10");

    // ---- CD11: unused upper bits of readout words are zero -----------------------------------
    $display("CD11: word zero-extension");
    default_sens(); do_clear();
    ev(regs(5, 31), PcBlkW - 1, MemBlkW - 1, 3); roll = 1'b1; step();
    for (int w = 0; w < NumWords; w++) begin
      logic [31:0] mask;
      mask = (w < NrSens) ? 32'((64'd1 << CntW) - 1) :
             (w == NrSens) ? 32'hFFFF_FFFF :
             (w == NrSens + 1) ? 32'((64'd1 << PcBlkW) - 1) :
             (w == NrSens + 2) ? 32'((64'd1 << MemBlkW) - 1) : 32'((64'd1 << CntW) - 1);
      checks++;
      if ((word[w] & ~mask) !== 32'h0) begin errs++; $display("FAIL CD11: word[%0d]=0x%0h has bits outside its field", w, word[w]); end
    end
    expect_w(NrSens + 1, 64'd1 << (PcBlkW - 1),  "CD11 top pc block");
    expect_w(NrSens + 2, 64'd1 << (MemBlkW - 1), "CD11 top mem block");

    // ---- CD12: reset clears everything even mid-interval --------------------------------------
    $display("CD12: asynchronous reset");
    ev(regs(5, 6), 1, 1, 2); step();
    ev(regs(5), 1, 1, 1); roll = 1'b1; step();
    ev(regs(5, 6), 1, 1, 2); step();
    #1 rst_n = 1'b0; #1;
    ref_reset();
    expect_all_zero("CD12 outputs cleared by async reset");
    @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);
    // the live data must have been cleared too: roll shows nothing
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_all_zero("CD12 live cleared by reset");

    // ======================================================================
    // CR1: random, compared every cycle
    // ======================================================================
    $display("CR1: random (150000 cycles)");
    default_sens(); do_clear();
    for (int c = 0; c < 150000; c++) begin
      logic [RegNum-1:0] r;
      r = '0;
      for (int k = 0; k < $urandom_range(0, 3); k++) r[$urandom_range(0, 11)] = 1'b1;   // dense on x0..x11
      if ($urandom_range(0, 15) == 0) r[$urandom_range(0, 31)] = 1'b1;
      reg_inc = r;
      pc_blk  = ($urandom_range(0, 3) == 0) ? (PcBlkW'(1) << $urandom_range(0, PcBlkW - 1)) : '0;
      mem_blk = ($urandom_range(0, 3) == 0) ? (MemBlkW'(1) << $urandom_range(0, MemBlkW - 1)) : '0;
      ret_cnt = RW'($urandom_range(0, 2));
      acc_en  = ($urandom_range(0, 19) != 0);
      roll    = ($urandom_range(0, 19) == 0);
      clear   = ($urandom_range(0, 149) == 0);
      if ($urandom_range(0, 99) == 0) begin                  // reprogram the table now and then
        for (int s = 0; s < NrSens; s++) begin
          sens[s].valid   = ($urandom_range(0, 3) != 0);
          sens[s].reg_idx = 5'($urandom_range(0, 11));
        end
        if (sens[0].reg_idx == sens[1].reg_idx && sens[0].valid && sens[1].valid) cv_dupslot++;
      end
      step();
    end

    // ======================================================================
    // CR2: long hot phase, no rolls -> saturation under random traffic
    // ======================================================================
    $display("CR2: random saturation phase (%0d cycles)", MAXC + 2000);
    default_sens(); do_clear();
    for (longint unsigned c = 0; c < MAXC + 2000; c++) begin
      reg_inc = '0;
      reg_inc[5]  = 1'b1;                       // every cycle -> must hit the limit
      reg_inc[10] = ($urandom_range(0, 1) == 1);
      reg_inc[$urandom_range(0, 31)] = 1'b1;
      pc_blk  = PcBlkW'($urandom());
      mem_blk = MemBlkW'($urandom());
      ret_cnt = RW'($urandom_range(0, 2));
      step();
    end
    ev('0, -1, -1, 0); roll = 1'b1; step();
    expect_w(0, MAXC, "CR2 x5 saturated (hit every one of 2^CntW+2000 cycles)");

    $display("coverage: rolls=%0d clears=%0d sat_intervals=%0d roll_without_acc=%0d dup_slots=%0d invalid_slot_hits=%0d",
             cv_roll, cv_clear, cv_sat, cv_roll_noacc, cv_dupslot, cv_invalid_hit);
    if (cv_roll < 5000)      begin errs++; $display("FAIL: coverage: too few rolls"); end
    if (cv_clear < 200)      begin errs++; $display("FAIL: coverage: too few clears"); end
    if (cv_sat < 3)          begin errs++; $display("FAIL: coverage: saturation rarely reached"); end
    if (cv_roll_noacc < 100) begin errs++; $display("FAIL: coverage: roll without acc_en rarely hit"); end
    if (cv_dupslot < 5)      begin errs++; $display("FAIL: coverage: duplicate sensitive slots rarely hit"); end
    if (cv_invalid_hit < 500) begin errs++; $display("FAIL: coverage: invalid slots with touched register rarely hit"); end

    $display("checks run: %0d", checks);
    if (errs == 0) $display("ALL DFDTMH_COUNTER_BANK TESTS PASSED");
    else           $display("DFDTMH_COUNTER_BANK TESTS FAILED: %0d error(s)", errs);
    $finish;
  end

  initial begin
    #(400_000_000);
    $display("TIMEOUT");
    $finish;
  end
endmodule