// -----------------------------------------------------------------------------
// MINIMAL ariane_pkg stub for STANDALONE testing of tmh_* and dfdtmh_* only.
// Superset of the SBTMH stub: every original enumerator keeps its value, new
// ones (FP load/store ops, CSR ops) are appended. NOT a replacement for CVA6's
// real ariane_pkg -- never compile this together with real CVA6.
// -----------------------------------------------------------------------------
package ariane_pkg;
  typedef enum logic [3:0] {
    ALU       = 4'd0,
    CTRL_FLOW = 4'd1,
    MULT      = 4'd2,
    CSR       = 4'd3,
    LOAD      = 4'd4,
    STORE     = 4'd5,
    FPU       = 4'd6,
    ACCEL     = 4'd7
  } fu_t;

  typedef enum logic [7:0] {
    ADD    = 8'd0,
    SUB    = 8'd1,
    XORL   = 8'd2,
    ORL    = 8'd3,
    ANDL   = 8'd4,
    MUL    = 8'd5,
    BRANCH = 8'd6,
    LD     = 8'd7,
    SD     = 8'd8,
    // --- added for DFDTMH (names match real CVA6 ariane_pkg::fu_op) ---
    FLD       = 8'd9,
    FLW       = 8'd10,
    FLH       = 8'd11,
    FLB       = 8'd12,
    FSD       = 8'd13,
    FSW       = 8'd14,
    FSH       = 8'd15,
    FSB       = 8'd16,
    CSR_WRITE = 8'd17,
    CSR_READ  = 8'd18
  } fu_op;
endpackage