// ------------------------------------------------------------------------------
// dfdtmh_ctrl_fsm.sv
// Monitoring-interval controller for DFDTMH (synthesizable)
//
// Defines the sampling interval, produces acc_en / roll, and manages the
// NEW / OVERRUN handshake with software.
// ------------------------------------------------------------------------------
module dfdtmh_ctrl_fsm
  import dfdtmh_pkg::*;
#(
  parameter int unsigned RetW = 2
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  // configuration (from dfdtmh_csr)
  input  logic                en_i,
  input  logic                oneshot_i,
  input  logic                mode_i,        // 0 = cycles, 1 = retired instructions
  input  logic [31:0]         intvl_i,       // >= 1
  input  logic                force_roll_i,  // 1-cycle pulse
  input  logic                clear_i,       // 1-cycle pulse
  input  logic                ack_i,         // 1-cycle pulse

  // from event generator
  input  logic [RetW-1:0]     retire_cnt_i,

  // to counter bank / CSR
  output dfdtmh_state_e       state_o,
  output logic                acc_en_o,
  output logic                roll_o,
  output logic                new_o,
  output logic                ovr_o,
  output logic [15:0]         seq_o
);

  dfdtmh_state_e state_q, state_d;
  logic [31:0]   icnt_q,  icnt_d;
  logic          new_q,   new_d;
  logic          ovr_q,   ovr_d;
  logic [15:0]   seq_q,   seq_d;

  logic          run_c;
  logic          roll_c;
  logic          expired;
  logic [32:0]   tick;
  logic [32:0]   icnt_next;

  // --------------------------------------------------------------------------
  // Next-state / control logic
  // --------------------------------------------------------------------------
  always_comb begin : p_next
    // default state transition
    unique case (state_q)
      IDLE:    state_d = en_i ? RUN : IDLE;
      RUN:     state_d = en_i ? RUN : IDLE;
      DONE:    state_d = ack_i ? (en_i ? RUN : IDLE) : DONE;
      default: state_d = IDLE;
    endcase

    if (clear_i)
      state_d = en_i ? RUN : IDLE;

    run_c = (state_d == RUN);

    // interval counter
    tick      = mode_i ? 33'(retire_cnt_i) : 33'd1;
    icnt_next = {1'b0, icnt_q} + tick;
    expired   = run_c && (icnt_next >= {1'b0, intvl_i});
    roll_c    = !clear_i && (force_roll_i || expired);

    if (clear_i || roll_c)
      icnt_d = '0;
    else if (run_c)
      icnt_d = icnt_next[31:0];
    else
      icnt_d = icnt_q;

    // one-shot: park in DONE after a roll
    if (roll_c && oneshot_i && run_c)
      state_d = DONE;

    // flags
    new_d = new_q;
    ovr_d = ovr_q;
    seq_d = seq_q;

    if (clear_i) begin
      new_d = 1'b0;
      ovr_d = 1'b0;
      seq_d = '0;
    end else begin
      if (ack_i) begin
        new_d = 1'b0;
        ovr_d = 1'b0;
      end
      if (roll_c) begin
        if (new_q && !ack_i)
          ovr_d = 1'b1;
        new_d = 1'b1;
        seq_d = seq_q + 16'd1;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Sequential
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= IDLE;
      icnt_q  <= '0;
      new_q   <= 1'b0;
      ovr_q   <= 1'b0;
      seq_q   <= '0;
    end else begin
      state_q <= state_d;
      icnt_q  <= icnt_d;
      new_q   <= new_d;
      ovr_q   <= ovr_d;
      seq_q   <= seq_d;
    end
  end

  assign state_o  = state_q;
  assign acc_en_o = run_c;
  assign roll_o   = roll_c;
  assign new_o    = new_q;
  assign ovr_o    = ovr_q;
  assign seq_o    = seq_q;

endmodule