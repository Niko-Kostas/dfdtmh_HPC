`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2026 01:28:51 AM
// Design Name: 
// Module Name: dfdtmh_counter_bank
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:// ------------------------------------------------------------------------------
// dfdtmh_counter_bank.sv
// LIVE / SHADOW double-buffered storage for DFDTMH (synthesizable)
//
//   * NrSens sensitive registers → full CntW-bit counters
//   * All other registers → 1-bit sticky “touched” bitmap
//   * PC and memory block bitmaps
//   * Retired-instruction counter
//
// On roll_i the post-increment LIVE values are copied to SHADOW and LIVE is
// zeroed on the same clock edge (no dead cycle).  clear_i has priority.
// Counters saturate; a sticky SAT flag is captured per interval.
// ------------------------------------------------------------------------------
module dfdtmh_counter_bank
  import dfdtmh_pkg::*;
#(
  parameter int unsigned RetW = 2
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  // events from dfdtmh_event_gen
  input  logic [RegNum-1:0]    reg_inc_i,
  input  logic [PcBlkW-1:0]    pc_blk_i,
  input  logic [MemBlkW-1:0]   mem_blk_i,
  input  logic [RetW-1:0]      retire_cnt_i,

  // sensitive-register table
  input  sens_table_t          sens_i,

  // control
  input  logic                 acc_en_i,
  input  logic                 roll_i,
  input  logic                 clear_i,

  // SHADOW read image (zero-extended to 32 bits)
  output logic [31:0]          word_o [NumWords],
  output logic                 shadow_sat_o
);

  // --------------------------------------------------------------------------
  // LIVE / SHADOW registers
  // --------------------------------------------------------------------------
  logic [CntW-1:0]    cnt_q  [NrSens], cnt_d  [NrSens];
  logic [RegNum-1:0]  rmap_q, rmap_d;
  logic [PcBlkW-1:0]  pmap_q, pmap_d;
  logic [MemBlkW-1:0] mmap_q, mmap_d;
  logic [CntW-1:0]    ret_q,  ret_d;
  logic               sat_q,  sat_d;

  logic [CntW-1:0]    scnt_q [NrSens], scnt_d [NrSens];
  logic [RegNum-1:0]  srmap_q, srmap_d;
  logic [PcBlkW-1:0]  spmap_q, spmap_d;
  logic [MemBlkW-1:0] smmap_q, smmap_d;
  logic [CntW-1:0]    sret_q,  sret_d;
  logic               ssat_q,  ssat_d;

  // --------------------------------------------------------------------------
  // Combinational accumulate (post-increment values)
  // --------------------------------------------------------------------------
  logic [CntW-1:0]    a_cnt [NrSens];
  logic [RegNum-1:0]  a_rmap;
  logic [PcBlkW-1:0]  a_pmap;
  logic [MemBlkW-1:0] a_mmap;
  logic [CntW-1:0]    a_ret;
  logic               a_sat;

  logic [RegNum-1:0]  sens_mask;

  always_comb begin : p_accumulate
    logic [CntW:0] sum;
    logic          inc;

    // build mask of currently sensitive registers
    for (int unsigned r = 0; r < RegNum; r++)
      sens_mask[r] = is_sensitive(5'(r), {>>{sens_i}});

    // default: hold
    a_sat  = sat_q;
    a_rmap = rmap_q;
    a_pmap = pmap_q;
    a_mmap = mmap_q;
    a_ret  = ret_q;
    for (int unsigned s = 0; s < NrSens; s++)
      a_cnt[s] = cnt_q[s];

    if (acc_en_i) begin
      // sensitive counters
      for (int unsigned s = 0; s < NrSens; s++) begin
        inc = sens_i[s].valid &&
              (int'(sens_i[s].reg_idx) < int'(RegNum)) &&
              reg_inc_i[sens_i[s].reg_idx];
        sum = {1'b0, cnt_q[s]} + {{CntW{1'b0}}, inc};
        if (sum[CntW]) begin
          a_cnt[s] = {CntW{1'b1}};
          a_sat    = 1'b1;
        end else begin
          a_cnt[s] = sum[CntW-1:0];
        end
      end

      // non-sensitive register bitmap
      a_rmap = rmap_q | (reg_inc_i & ~sens_mask);

      // PC / memory bitmaps
      a_pmap = pmap_q | pc_blk_i;
      a_mmap = mmap_q | mem_blk_i;

      // retire counter
      sum = {1'b0, ret_q} + {{(CntW+1-RetW){1'b0}}, retire_cnt_i};
      if (sum[CntW]) begin
        a_ret = {CntW{1'b1}};
        a_sat = 1'b1;
      end else begin
        a_ret = sum[CntW-1:0];
      end
    end
  end

  // --------------------------------------------------------------------------
  // Roll / clear next-state
  // --------------------------------------------------------------------------
  always_comb begin : p_roll
    // default: LIVE follows accumulate, SHADOW holds
    for (int unsigned s = 0; s < NrSens; s++) begin
      cnt_d[s]  = a_cnt[s];
      scnt_d[s] = scnt_q[s];
    end
    rmap_d  = a_rmap;  pmap_d  = a_pmap;  mmap_d  = a_mmap;
    ret_d   = a_ret;   sat_d   = a_sat;
    srmap_d = srmap_q; spmap_d = spmap_q; smmap_d = smmap_q;
    sret_d  = sret_q;  ssat_d  = ssat_q;

    if (clear_i) begin
      for (int unsigned s = 0; s < NrSens; s++) begin
        cnt_d[s]  = '0;
        scnt_d[s] = '0;
      end
      rmap_d  = '0; pmap_d  = '0; mmap_d  = '0; ret_d  = '0; sat_d  = 1'b0;
      srmap_d = '0; spmap_d = '0; smmap_d = '0; sret_d = '0; ssat_d = 1'b0;
    end else if (roll_i) begin
      for (int unsigned s = 0; s < NrSens; s++) begin
        scnt_d[s] = a_cnt[s];
        cnt_d[s]  = '0;
      end
      srmap_d = a_rmap; rmap_d = '0;
      spmap_d = a_pmap; pmap_d = '0;
      smmap_d = a_mmap; mmap_d = '0;
      sret_d  = a_ret;  ret_d  = '0;
      ssat_d  = a_sat;  sat_d  = 1'b0;
    end
  end

  // --------------------------------------------------------------------------
  // Sequential update
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int unsigned s = 0; s < NrSens; s++) begin
        cnt_q[s]  <= '0;
        scnt_q[s] <= '0;
      end
      rmap_q  <= '0; pmap_q  <= '0; mmap_q  <= '0; ret_q  <= '0; sat_q  <= 1'b0;
      srmap_q <= '0; spmap_q <= '0; smmap_q <= '0; sret_q <= '0; ssat_q <= 1'b0;
    end else begin
      cnt_q   <= cnt_d;    scnt_q  <= scnt_d;
      rmap_q  <= rmap_d;   pmap_q  <= pmap_d;  mmap_q  <= mmap_d;
      ret_q   <= ret_d;    sat_q   <= sat_d;
      srmap_q <= srmap_d;  spmap_q <= spmap_d; smmap_q <= smmap_d;
      sret_q  <= sret_d;   ssat_q  <= ssat_d;
    end
  end

  // --------------------------------------------------------------------------
  // Read image (SHADOW only)
  // --------------------------------------------------------------------------
  always_comb begin : p_words
    for (int unsigned w = 0; w < NumWords; w++)
      word_o[w] = '0;
    for (int unsigned s = 0; s < NrSens; s++)
      word_o[s] = 32'(scnt_q[s]);
    word_o[NrSens]     = 32'(srmap_q);
    word_o[NrSens + 1] = 32'(spmap_q);
    word_o[NrSens + 2] = 32'(smmap_q);
    word_o[NrSens + 3] = 32'(sret_q);
  end

  assign shadow_sat_o = ssat_q;

endmodule
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module dfdtmh_counter_bank(

    );
endmodule
