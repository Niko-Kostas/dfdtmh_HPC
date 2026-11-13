// ------------------------------------------------------------------------------
// tb_dfdtmh_fsm.sv -- self-checking testbench for dfdtmh_ctrl_fsm
//
// Layer 1 (FD1..FD10): directed tests, expected cycle timing HAND-COMPUTED.
// Layer 2 (FR1/FR2)  : random + seq-wrap run compared EVERY CYCLE with a reference
//                      model (state, acc_en, roll, new, overrun, seq).
//
// acc_en_o and roll_o are combinational in this cycle's inputs, so they are sampled
// just before the clock edge; state/new/overrun/seq are registered and are sampled
// after it. Inputs change right after a negedge.
// ------------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dfdtmh_fsm;
  import dfdtmh_pkg::*;

  localparam int unsigned RW = 2;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  logic          en, oneshot, mode, force_roll, clear, ack;
  logic [31:0]   intvl;
  logic [RW-1:0] ret;
  dfdtmh_state_e state;
  logic          acc_en, roll, new_f, ovr_f;
  logic [15:0]   seq;

  dfdtmh_ctrl_fsm #(.RetW(RW)) dut (
      .clk_i(clk), .rst_ni(rst_n),
      .en_i(en), .oneshot_i(oneshot), .mode_i(mode), .intvl_i(intvl),
      .force_roll_i(force_roll), .clear_i(clear), .ack_i(ack),
      .retire_cnt_i(ret),
      .state_o(state), .acc_en_o(acc_en), .roll_o(roll),
      .new_o(new_f), .ovr_o(ovr_f), .seq_o(seq)
  );

  // ---------------- reference ---------------------------------------------------
  int          r_state;      // 0 IDLE 1 RUN 2 DONE
  longint      r_icnt;
  bit          r_new, r_ovr;
  int          r_seq;
  bit          x_acc, x_roll;

  int errs = 0;
  int checks = 0;
  int cv_roll, cv_ovr, cv_done, cv_force_idle, cv_force_done, cv_clear_roll, cv_ack_roll, cv_instr, cv_pause, cv_wrap;

  task automatic ref_reset();
    r_state = 0; r_icnt = 0; r_new = 0; r_ovr = 0; r_seq = 0; x_acc = 0; x_roll = 0;
  endtask

  task automatic ref_step();
    int ns;
    bit run, expired, rl, old_new;
    longint tick;
    case (r_state)
      0:       ns = en ? 1 : 0;
      1:       ns = en ? 1 : 0;
      default: ns = ack ? (en ? 1 : 0) : 2;
    endcase
    if (clear) ns = en ? 1 : 0;
    run     = (ns == 1);
    tick    = mode ? longint'(ret) : 1;
    expired = run && ((r_icnt + tick) >= longint'(intvl));
    rl      = !clear && (force_roll || expired);
    if (clear || rl) r_icnt = 0; else if (run) r_icnt = r_icnt + tick;
    if (rl && oneshot && run) ns = 2;
    old_new = r_new;
    if (clear) begin r_new = 0; r_ovr = 0; r_seq = 0; end
    else begin
      if (ack) begin r_new = 0; r_ovr = 0; end
      if (rl) begin
        if (old_new && !ack) begin r_ovr = 1; cv_ovr++; end
        r_new = 1;
        r_seq = (r_seq + 1) & 32'hFFFF;
        cv_roll++;
        if (force_roll && r_state == 0) cv_force_idle++;
        if (force_roll && r_state == 2) cv_force_done++;
        if (ack) cv_ack_roll++;
        if (ns == 2 && r_state != 2) cv_done++;
      end
    end
    if (clear && (force_roll || expired)) cv_clear_roll++;
    if (mode && run) cv_instr++;
    if (r_state == 1 && !en) cv_pause++;
    if (rl && r_seq == 0) cv_wrap++;
    x_acc = run; x_roll = rl;
    r_state = ns;
  endtask

  task automatic fail(input string m);
    errs++;
    $display("[%0t] FAIL: %s", $time, m);
    if (errs >= 25) begin $display("too many errors"); $finish; end
  endtask

  // one clock: comb outputs checked before the edge, registered ones after
  task automatic step();
    ref_step();
    #1;
    checks += 2;
    if (acc_en !== x_acc)  fail($sformatf("acc_en_o=%b expected %b", acc_en, x_acc));
    if (roll   !== x_roll) fail($sformatf("roll_o=%b expected %b", roll, x_roll));
    @(posedge clk);
    @(negedge clk);
    checks += 4;
    if (int'(state) !== r_state) fail($sformatf("state=%0d expected %0d", state, r_state));
    if (new_f !== r_new)         fail($sformatf("new=%b expected %b", new_f, r_new));
    if (ovr_f !== r_ovr)         fail($sformatf("overrun=%b expected %b", ovr_f, r_ovr));
    if (int'(seq) !== r_seq)     fail($sformatf("seq=%0d expected %0d", seq, r_seq));
    force_roll = 1'b0; clear = 1'b0; ack = 1'b0; ret = '0;
  endtask

  // absolute expectations (independent of the reference)
  task automatic expect_state(input int st, input bit nw, input bit ov, input int sq, input string what);
    checks++;
    if (int'(state) !== st || new_f !== nw || ovr_f !== ov || int'(seq) !== sq)
      fail($sformatf("%s: state=%0d new=%b ovr=%b seq=%0d, expected state=%0d new=%b ovr=%b seq=%0d",
                     what, state, new_f, ovr_f, seq, st, nw, ov, sq));
  endtask

  // run n cycles, returning the 0-based index of the cycle in which roll_o fired (-1 if none)
  task automatic run_n(input int n, output int roll_at, input bit with_ret = 1'b0);
    roll_at = -1;
    for (int i = 0; i < n; i++) begin
      if (with_ret) ret = RW'(2);
      ref_step();
      #1;
      if (roll && roll_at < 0) roll_at = i;
      checks += 2;
      if (acc_en !== x_acc)  fail($sformatf("acc_en_o=%b expected %b", acc_en, x_acc));
      if (roll   !== x_roll) fail($sformatf("roll_o=%b expected %b", roll, x_roll));
      @(posedge clk);
      @(negedge clk);
      force_roll = 1'b0; clear = 1'b0; ack = 1'b0; ret = '0;
    end
  endtask

  task automatic cfg(input bit e, input bit os, input bit md, input int unsigned iv);
    en = e; oneshot = os; mode = md; intvl = 32'(iv);
  endtask

  initial begin
    int at;
    rst_n = 1'b0; en = 0; oneshot = 0; mode = 0; intvl = 32'd1000;
    force_roll = 0; clear = 0; ack = 0; ret = '0;
    cv_roll = 0; cv_ovr = 0; cv_done = 0; cv_force_idle = 0; cv_force_done = 0;
    cv_clear_roll = 0; cv_ack_roll = 0; cv_instr = 0; cv_pause = 0; cv_wrap = 0;
    ref_reset();
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    @(negedge clk);

    // ---- FD1: reset -----------------------------------------------------------
    $display("FD1: reset");
    expect_state(0, 0, 0, 0, "FD1");
    checks++; if (acc_en !== 1'b0 || roll !== 1'b0) fail("FD1 idle must not accumulate or roll");

    // ---- FD2: cycle mode, interval 4, free running ----------------------------
    //   en=1 from cycle 0: accumulation starts in cycle 0 (next-state derived),
    //   roll in cycle 3 (4th cycle), then again in cycle 7, ...
    $display("FD2: cycle-mode interval=4");
    cfg(1, 0, 0, 4);
    #1; checks++; if (acc_en !== 1'b1) fail("FD2 acc_en must be high in the very cycle EN is seen");
    run_n(4, at);
    checks++; if (at !== 3) fail($sformatf("FD2 first roll in cycle index %0d, expected 3", at));
    expect_state(1, 1, 0, 1, "FD2 after first interval");
    run_n(8, at);
    checks++; if (at !== 3) fail($sformatf("FD2 second roll in cycle index %0d, expected 3 (period 4)", at));
    expect_state(1, 1, 1, 3, "FD2 two more rolls, never acked -> overrun");

    // ---- FD3: ack clears NEW/OVERRUN -------------------------------------------
    $display("FD3: ack");
    ack = 1'b1; cfg(1, 0, 0, 4);
    run_n(1, at);
    expect_state(1, 0, 0, 3, "FD3 ack clears new+overrun, seq kept");

    // ---- FD4: ack in the same cycle as a roll consumes the OLD sample: no overrun --------
    $display("FD4: ack + roll same cycle");
    // state: RUN, 1 cycle into a 4-cycle interval, NEW=0, seq=3
    run_n(3, at);                 // roll #4 -> NEW=1 (precondition for the test)
    checks++; if (at !== 2) fail("FD4 precondition: roll in third cycle");
    expect_state(1, 1, 0, 4, "FD4 precondition: NEW set, no overrun");
    run_n(3, at);                 // 3 cycles of the next interval
    ack = 1'b1;
    run_n(1, at);                 // roll cycle WITH ack while NEW is still 1
    checks++; if (at !== 0) fail("FD4 roll expected in the ack cycle");
    expect_state(1, 1, 0, 5, "FD4 roll+ack: old sample consumed, NEW set again, NO overrun");
    run_n(3, at);
    run_n(1, at);                 // roll with NEW set and no ack -> overrun
    expect_state(1, 1, 1, 6, "FD4 roll without ack while NEW set -> overrun");

    // ---- FD5: clear ---------------------------------------------------------------
    $display("FD5: clear");
    clear = 1'b1; force_roll = 1'b1;     // clear must beat a same-cycle snapshot
    run_n(1, at);
    checks++; if (at !== -1) fail("FD5 clear must suppress roll_o");
    expect_state(1, 0, 0, 0, "FD5 clear: flags and seq reset, still RUN because en=1");
    run_n(3, at);
    checks++; if (at !== -1) fail("FD5 interval counter must restart after clear (3 cycles, no roll yet)");
    run_n(1, at);
    checks++; if (at !== 0) fail("FD5 roll exactly 4 cycles after the clear cycle's successor");

    // ---- FD6: pause / resume keeps the partial interval ------------------------------
    $display("FD6: en drop and resume");
    cfg(1, 0, 0, 10); clear = 1'b1; run_n(1, at);
    run_n(3, at);                          // 3 of 10 cycles done
    cfg(0, 0, 0, 10);
    run_n(1, at);
    expect_state(0, 0, 0, 0, "FD6 en=0 -> IDLE");
    run_n(5, at);
    checks++; if (at !== -1) fail("FD6 no roll while paused");
    cfg(1, 0, 0, 10);
    run_n(6, at);
    checks++; if (at !== -1) fail("FD6 resumed: 3+6 = 9 cycles, no roll yet");
    run_n(1, at);
    checks++; if (at !== 0) fail("FD6 resumed: roll at the 10th accumulated cycle");

    // ---- FD7: one-shot -------------------------------------------------------------------
    $display("FD7: one-shot");
    cfg(1, 1, 0, 3); clear = 1'b1; run_n(1, at);
    run_n(3, at);
    checks++; if (at !== 2) fail("FD7 roll in third cycle");
    expect_state(2, 1, 0, 1, "FD7 DONE after one-shot interval");
    run_n(5, at);
    expect_state(2, 1, 0, 1, "FD7 DONE holds, no further rolls");
    checks++; if (acc_en !== 1'b0) fail("FD7 acc_en must be low in DONE");
    ack = 1'b1; cfg(0, 1, 0, 3);          // ack with en=0 -> IDLE
    run_n(1, at);
    expect_state(0, 0, 0, 1, "FD7 ack with en=0 -> IDLE");
    cfg(1, 1, 0, 3);
    run_n(3, at);                          // roll again -> DONE
    expect_state(2, 1, 0, 2, "FD7b second one-shot");
    ack = 1'b1; run_n(1, at);              // ack with en=1 -> RUN (accumulating that cycle)
    expect_state(1, 0, 0, 2, "FD7b ack with en=1 -> RUN");

    // ---- FD8: force roll in every state ---------------------------------------------------
    $display("FD8: snapshot");
    cfg(0, 0, 0, 100); clear = 1'b1; run_n(1, at); run_n(1, at);
    force_roll = 1'b1; run_n(1, at);
    checks++; if (at !== 0) fail("FD8 snapshot in IDLE must roll");
    expect_state(0, 1, 0, 1, "FD8 snapshot from IDLE");
    cfg(1, 0, 0, 100); clear = 1'b1; run_n(1, at);
    run_n(2, at);
    force_roll = 1'b1; run_n(1, at);
    checks++; if (at !== 0) fail("FD8 snapshot in RUN must roll");
    expect_state(1, 1, 0, 1, "FD8 snapshot from RUN keeps running");
    run_n(2, at);
    checks++; if (at !== -1) fail("FD8 interval counter restarted by the snapshot");
    cfg(1, 1, 0, 100); clear = 1'b1; run_n(1, at);
    force_roll = 1'b1; run_n(1, at);        // RUN + oneshot + snapshot -> DONE
    expect_state(2, 1, 0, 1, "FD8 snapshot in one-shot RUN -> DONE");
    force_roll = 1'b1; run_n(1, at);        // snapshot in DONE: rolls, stays DONE
    expect_state(2, 1, 1, 2, "FD8 snapshot in DONE: rolls again, overrun");

    // ---- FD9: instruction mode, 2 retire per cycle ------------------------------------------
    $display("FD9: instruction mode");
    cfg(1, 0, 1, 5); clear = 1'b1; run_n(1, at);
    run_n(3, at);                           // retire 0 -> no progress
    checks++; if (at !== -1) fail("FD9 no retirement, no progress");
    run_n(3, at, 1'b1);                     // 2+2=4 <5, 6>=5 -> roll in third cycle
    checks++; if (at !== 2) fail($sformatf("FD9 roll in cycle %0d, expected 2 (overshoot by 1)", at));
    cfg(1, 0, 1, 4); clear = 1'b1; run_n(1, at);
    run_n(2, at, 1'b1);                     // 2+2 = 4 >= 4 exactly on the second cycle
    checks++; if (at !== 1) fail("FD9 exact hit: roll in second cycle");

    // ---- FD10: interval of 1 -----------------------------------------------------------------
    $display("FD10: interval=1");
    cfg(1, 0, 0, 1); clear = 1'b1; run_n(1, at);
    run_n(5, at);
    expect_state(1, 1, 1, 5, "FD10 rolls every cycle");

    // ---- FR1: random --------------------------------------------------------------------------
    $display("FR1: random (150000 cycles)");
    for (int c = 0; c < 150000; c++) begin
      int unsigned iv [8];
      iv = '{1, 1, 2, 3, 5, 8, 13, 40};
      if ($urandom_range(0, 24) == 0) begin
        en = ($urandom_range(0, 9) < 8);
        oneshot = ($urandom_range(0, 4) == 0);
        mode = ($urandom_range(0, 9) < 3);
        intvl = 32'(iv[$urandom_range(0, 7)]);
      end
      force_roll = ($urandom_range(0, 29) == 0);
      clear      = ($urandom_range(0, 59) == 0);
      ack        = ($urandom_range(0, 7) == 0);
      ret        = RW'($urandom_range(0, 2));
      step();
    end

    // ---- FR2: sequence counter wraps at 16 bits ------------------------------------------------
    $display("FR2: seq wrap");
    cfg(1, 0, 0, 1); clear = 1'b1; step();
    for (int c = 0; c < 65540; c++) step();
    checks++; if (cv_wrap == 0) fail("FR2 seq never wrapped");

    $display("coverage: rolls=%0d overruns=%0d done=%0d force_idle=%0d force_done=%0d clear_roll=%0d ack_roll=%0d instr=%0d pause=%0d wraps=%0d",
             cv_roll, cv_ovr, cv_done, cv_force_idle, cv_force_done, cv_clear_roll, cv_ack_roll, cv_instr, cv_pause, cv_wrap);
    if (cv_roll < 3000)      begin errs++; $display("FAIL: coverage: too few rolls"); end
    if (cv_ovr < 100)        begin errs++; $display("FAIL: coverage: overrun rarely hit"); end
    if (cv_done < 20)        begin errs++; $display("FAIL: coverage: one-shot DONE rarely hit"); end
    if (cv_force_idle < 10)  begin errs++; $display("FAIL: coverage: snapshot in IDLE rarely hit"); end
    if (cv_force_done < 5)   begin errs++; $display("FAIL: coverage: snapshot in DONE rarely hit"); end
    if (cv_clear_roll < 5)   begin errs++; $display("FAIL: coverage: clear-with-roll rarely hit"); end
    if (cv_ack_roll < 5)     begin errs++; $display("FAIL: coverage: ack-with-roll rarely hit"); end
    if (cv_instr < 1000)     begin errs++; $display("FAIL: coverage: instruction mode rarely hit"); end
    if (cv_pause < 20)       begin errs++; $display("FAIL: coverage: pause rarely hit"); end

    $display("checks run: %0d", checks);
    if (errs == 0) $display("ALL DFDTMH_FSM TESTS PASSED");
    else           $display("DFDTMH_FSM TESTS FAILED: %0d error(s)", errs);
    $finish;
  end
endmodule