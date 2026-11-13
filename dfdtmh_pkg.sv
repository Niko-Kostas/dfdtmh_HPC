// ------------------------------------------------------------------------------
// dfdtmh_pkg.sv
// Package for Design-for-Debug inspired Tailor-Made HPC (DFDTMH)
// Synthesizable shared definitions.
// ------------------------------------------------------------------------------
package dfdtmh_pkg;

  // ---------------------------------------------------------------------------
  // Parameters (override from the top-level if needed)
  // ---------------------------------------------------------------------------
  parameter int unsigned NrSens       = 4;          // Number of sensitive registers
  parameter int unsigned CntW         = 16;         // Counter width
  parameter int unsigned RegNum       = 32;         // Architectural registers
  parameter int unsigned PcBlkW       = 16;         // PC bitmap width
  parameter int unsigned MemBlkW      = 16;         // Memory bitmap width
  parameter int unsigned PcShift      = 2;          // Address bit-slice shift for PC blocks
  parameter int unsigned MemShift     = 4;          // Address bit-slice shift for memory blocks

  // Derived
  parameter int unsigned NumWords     = NrSens + 4; // counters + reg/PC/mem bitmaps + retire count
  parameter int unsigned SensIdxW     = (NrSens > 1) ? $clog2(NrSens) : 1;

  // ---------------------------------------------------------------------------
  // CSR addresses (machine custom)
  // ---------------------------------------------------------------------------
  localparam logic [11:0] CSR_DFD_CTRL   = 12'h7C8;
  localparam logic [11:0] CSR_DFD_STATUS = 12'h7C9;
  localparam logic [11:0] CSR_DFD_INTVL  = 12'h7CA;
  localparam logic [11:0] CSR_DFD_SEL    = 12'h7CB;
  localparam logic [11:0] CSR_DFD_DATA   = 12'h7CC;
  localparam logic [11:0] CSR_DFD_SENS   = 12'h7CD;
  localparam logic [11:0] CSR_DFD_INFO   = 12'h7CE;

  // ---------------------------------------------------------------------------
  // FSM states
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    IDLE  = 2'b00,
    RUN   = 2'b01,
    DONE  = 2'b10
  } dfdtmh_state_e;

  // ---------------------------------------------------------------------------
  // Sensitive register table entry
  // DFD_SENS layout: slot s at [6*s +: 6] = {valid, reg[4:0]}
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        valid;
    logic [4:0]  reg_idx;
  } sens_entry_t;

  typedef sens_entry_t [NrSens-1:0] sens_table_t;

  // ---------------------------------------------------------------------------
  // Status bits (DFD_STATUS) - must be exactly 32 bits
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [15:0] seq;        // [31:16]
    logic [10:0] reserved;   // [15:5]  keep zero
    logic        sat;        // [4]
    logic        overrun;    // [3]
    logic        new_sample; // [2]
    logic [1:0]  state;      // [1:0]
  } dfdtmh_status_t;

  // ---------------------------------------------------------------------------
  // Control register bits (DFD_CTRL)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic ack;       // [5] pulse
    logic clear;     // [4] pulse
    logic snapshot;  // [3] pulse
    logic mode;      // [2] 0 = cycles, 1 = instructions
    logic oneshot;   // [1]
    logic en;        // [0]
  } dfdtmh_ctrl_t;

  // ---------------------------------------------------------------------------
  // Helper: is register r sensitive according to the table?
  // Synthesizable (unrolled at elaboration for typical NrSens).
  // ---------------------------------------------------------------------------
  // table_raw: slot s at bits [6*s +: 6] = {valid, reg_idx[4:0]}
  function automatic logic is_sensitive;
    input logic [4:0]           reg_idx;
    input logic [6*NrSens-1:0]  table_raw;
    integer i;
    begin
      is_sensitive = 1'b0;
      for (i = 0; i < NrSens; i = i + 1) begin
        if (table_raw[6*i + 5] && (table_raw[6*i +: 5] == reg_idx))
          is_sensitive = 1'b1;
      end
    end
  endfunction

endpackage