// ------------------------------------------------------------------------------
// cva6_dfdtmh_unit.sv
// Top-level DFDTMH wrapper (synthesizable)
//
// Instantiates:
//   dfdtmh_event_gen → dfdtmh_ctrl_fsm → dfdtmh_counter_bank → dfdtmh_csr
// ------------------------------------------------------------------------------
module cva6_dfdtmh_unit
  import dfdtmh_pkg::*;
#(
  parameter int unsigned NR_COMMIT_PORTS    = 2,
  parameter type         scoreboard_entry_t = logic,
  parameter int unsigned CsrDataWidth       = 64,
  parameter int unsigned MemAddrWidth       = 56,
  parameter int unsigned DefaultInterval    = 1000,
  parameter sens_table_t SensInit           = '0,
  parameter int unsigned RetW               =
      (NR_COMMIT_PORTS <= 1) ? 1 : $clog2(NR_COMMIT_PORTS + 1)
) (
  input  logic                                       clk_i,
  input  logic                                       rst_ni,

  input  scoreboard_entry_t [NR_COMMIT_PORTS-1:0]    commit_instr_i,
  input  logic              [NR_COMMIT_PORTS-1:0]    commit_ack_i,
  input  logic [MemAddrWidth-1:0]                    mem_addr_i,

  // generic CSR bus
  input  logic [11:0]                                addr_i,
  input  logic                                       we_i,
  input  logic [CsrDataWidth-1:0]                    data_i,
  output logic [CsrDataWidth-1:0]                    csr_rdata_o,
  output logic                                       csr_addr_hit_o,

  // optional hardware strobe
  output logic                                       interval_done_o
);

  // event datapath
  logic [RegNum-1:0]  reg_inc;
  logic               pc_set, mem_set, retire;
  logic [PcBlkW-1:0]  pc_blk;
  logic [MemBlkW-1:0] mem_blk;
  logic [RetW-1:0]    retire_cnt;

  // control
  logic               en, oneshot, mode, force_roll, clear, ack;
  logic [31:0]        intvl;
  sens_table_t        sens;
  dfdtmh_state_e      state;
  logic               acc_en, roll, new_f, ovr_f, sat_f;
  logic [15:0]        seq;
  logic [31:0]        words [NumWords];

  // --------------------------------------------------------------------------
  // Event generator
  // --------------------------------------------------------------------------
  dfdtmh_event_gen #(
    .NR_COMMIT_PORTS    (NR_COMMIT_PORTS),
    .scoreboard_entry_t (scoreboard_entry_t),
    .MemAddrWidth       (MemAddrWidth),
    .RetW               (RetW)
  ) i_event_gen (
    .commit_instr_i (commit_instr_i),
    .commit_ack_i   (commit_ack_i),
    .mem_addr_i     (mem_addr_i),
    .reg_inc_o      (reg_inc),
    .pc_set_o       (pc_set),
    .pc_blk_o       (pc_blk),
    .mem_set_o      (mem_set),
    .mem_blk_o      (mem_blk),
    .retire_o       (retire),
    .retire_cnt_o   (retire_cnt)
  );

  // --------------------------------------------------------------------------
  // Interval FSM
  // --------------------------------------------------------------------------
  dfdtmh_ctrl_fsm #(
    .RetW (RetW)
  ) i_fsm (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .en_i         (en),
    .oneshot_i    (oneshot),
    .mode_i       (mode),
    .intvl_i      (intvl),
    .force_roll_i (force_roll),
    .clear_i      (clear),
    .ack_i        (ack),
    .retire_cnt_i (retire_cnt),
    .state_o      (state),
    .acc_en_o     (acc_en),
    .roll_o       (roll),
    .new_o        (new_f),
    .ovr_o        (ovr_f),
    .seq_o        (seq)
  );

  // --------------------------------------------------------------------------
  // Counter / bitmap bank
  // --------------------------------------------------------------------------
  dfdtmh_counter_bank #(
    .RetW (RetW)
  ) i_bank (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .reg_inc_i    (reg_inc),
    .pc_blk_i     (pc_blk),
    .mem_blk_i    (mem_blk),
    .retire_cnt_i (retire_cnt),
    .sens_i       (sens),
    .acc_en_i     (acc_en),
    .roll_i       (roll),
    .clear_i      (clear),
    .word_o       (words),
    .shadow_sat_o (sat_f)
  );

  // --------------------------------------------------------------------------
  // CSR interface
  // --------------------------------------------------------------------------
  dfdtmh_csr #(
    .CsrDataWidth    (CsrDataWidth),
    .DefaultInterval (DefaultInterval),
    .SensInit        (SensInit)
  ) i_csr (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .addr_i         (addr_i),
    .we_i           (we_i),
    .data_i         (data_i),
    .csr_rdata_o    (csr_rdata_o),
    .csr_addr_hit_o (csr_addr_hit_o),
    .en_o           (en),
    .oneshot_o      (oneshot),
    .mode_o         (mode),
    .intvl_o        (intvl),
    .force_roll_o   (force_roll),
    .clear_o        (clear),
    .ack_o          (ack),
    .sens_o         (sens),
    .state_i        (state),
    .new_i          (new_f),
    .ovr_i          (ovr_f),
    .sat_i          (sat_f),
    .seq_i          (seq),
    .word_i         (words)
  );

  assign interval_done_o = roll;

endmodule