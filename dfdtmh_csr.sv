// ------------------------------------------------------------------------------
// dfdtmh_csr.sv
// CSR window 0x7C8-0x7CE for DFDTMH (synthesizable)
// ------------------------------------------------------------------------------
module dfdtmh_csr
  import dfdtmh_pkg::*;
#(
  parameter int unsigned CsrDataWidth    = 64,
  parameter int unsigned DefaultInterval = 1000,
  parameter sens_table_t SensInit        = '0
) (
  input  logic                      clk_i,
  input  logic                      rst_ni,

  // generic CSR bus
  input  logic [11:0]               addr_i,
  input  logic                      we_i,
  input  logic [CsrDataWidth-1:0]   data_i,
  output logic [CsrDataWidth-1:0]   csr_rdata_o,
  output logic                      csr_addr_hit_o,

  // configuration / pulses to the FSM and counter bank
  output logic                      en_o,
  output logic                      oneshot_o,
  output logic                      mode_o,
  output logic [31:0]               intvl_o,
  output logic                      force_roll_o,
  output logic                      clear_o,
  output logic                      ack_o,
  output sens_table_t               sens_o,

  // status / data in
  input  dfdtmh_state_e             state_i,
  input  logic                      new_i,
  input  logic                      ovr_i,
  input  logic                      sat_i,
  input  logic [15:0]               seq_i,
  input  logic [31:0]               word_i [NumWords]
);

  localparam logic [7:0] DFD_INFO_VERSION = 8'd1;
  localparam int unsigned SelW = (NumWords <= 1) ? 1 : $clog2(NumWords);

  logic              en_q, oneshot_q, mode_q;
  logic [31:0]       intvl_q;
  logic [SelW-1:0]   sel_q;
  sens_table_t       sens_q;

  // ---- address decode -------------------------------------------------------
  logic hit_ctrl, hit_status, hit_intvl, hit_sel, hit_data, hit_sens, hit_info;

  assign hit_ctrl   = (addr_i == CSR_DFD_CTRL);
  assign hit_status = (addr_i == CSR_DFD_STATUS);
  assign hit_intvl  = (addr_i == CSR_DFD_INTVL);
  assign hit_sel    = (addr_i == CSR_DFD_SEL);
  assign hit_data   = (addr_i == CSR_DFD_DATA);
  assign hit_sens   = (addr_i == CSR_DFD_SENS);
  assign hit_info   = (addr_i == CSR_DFD_INFO);

  assign csr_addr_hit_o = hit_ctrl | hit_status | hit_intvl | hit_sel |
                          hit_data | hit_sens  | hit_info;

  // ---- write decode ---------------------------------------------------------
  dfdtmh_ctrl_t ctrl_wr;
  assign ctrl_wr = dfdtmh_ctrl_t'(data_i[$bits(dfdtmh_ctrl_t)-1:0]);

  assign force_roll_o = we_i && hit_ctrl && ctrl_wr.snapshot;
  assign ack_o        = we_i && hit_ctrl && ctrl_wr.ack;
  assign clear_o      = (we_i && hit_ctrl && ctrl_wr.clear) ||
                        (we_i && hit_sens);

  // ---- WARL helpers ---------------------------------------------------------
  logic [31:0]     intvl_wr;
  logic [SelW-1:0] sel_wr;

  always_comb begin
    intvl_wr = data_i[31:0];
    if (intvl_wr == 32'd0)
      intvl_wr = 32'd1;

    // Compare the FULL write data at full width, then narrow.
    // (SelW'(NumWords) truncates to 0 when NumWords is a power of two.)
    if (data_i > CsrDataWidth'(NumWords - 1))
      sel_wr = SelW'(NumWords - 1);
    else
      sel_wr = data_i[SelW-1:0];
  end

  // ---- sequential configuration registers -----------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      en_q      <= 1'b0;
      oneshot_q <= 1'b0;
      mode_q    <= 1'b0;
      intvl_q   <= 32'(DefaultInterval);
      sel_q     <= '0;
      sens_q    <= SensInit;
    end else if (we_i) begin
      if (hit_ctrl) begin
        en_q      <= ctrl_wr.en;
        oneshot_q <= ctrl_wr.oneshot;
        mode_q    <= ctrl_wr.mode;
      end
      if (hit_intvl) intvl_q <= intvl_wr;
      if (hit_sel)   sel_q   <= sel_wr;
      if (hit_sens)  sens_q  <= sens_table_t'(data_i[$bits(sens_table_t)-1:0]);
    end
  end

  assign en_o      = en_q;
  assign oneshot_o = oneshot_q;
  assign mode_o    = mode_q;
  assign intvl_o   = intvl_q;
  assign sens_o    = sens_q;

  // ---- read mux -------------------------------------------------------------
  dfdtmh_status_t status_w;
  logic [31:0]    info_w;

  always_comb begin
    status_w            = '0;
    status_w.state      = state_i;
    status_w.new_sample = new_i;
    status_w.overrun    = ovr_i;
    status_w.sat        = sat_i;
    status_w.seq        = seq_i;

    info_w = {8'h00, 8'(NumWords), 8'(NrSens), DFD_INFO_VERSION};
  end

  always_comb begin
    csr_rdata_o = '0;
    unique case (1'b1)
      hit_ctrl:   csr_rdata_o = CsrDataWidth'({mode_q, oneshot_q, en_q});
      hit_status: csr_rdata_o = CsrDataWidth'(status_w);
      hit_intvl:  csr_rdata_o = CsrDataWidth'(intvl_q);
      hit_sel:    csr_rdata_o = CsrDataWidth'(sel_q);
      hit_data:   csr_rdata_o = CsrDataWidth'(word_i[sel_q]);
      hit_sens:   csr_rdata_o = CsrDataWidth'(sens_q);
      hit_info:   csr_rdata_o = CsrDataWidth'(info_w);
      default:    csr_rdata_o = '0;
    endcase
  end

endmodule