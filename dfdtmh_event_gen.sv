// ------------------------------------------------------------------------------
// dfdtmh_event_gen.sv
// Combinational event generator for DFDTMH
// Decodes retired instructions and produces register touches + PC/mem block sets.
//
// Based on the original hand-written version. Changes (all marked [v2]):
//   [v2] scoreboard_entry_t is a module parameter type (repo pattern, same as
//        cva6_tmh_unit); the global type / riscv::VLEN do not exist in CVA6.
//   [v2] mem_addr_i width is a parameter (MemAddrWidth = CVA6 PLEN), no riscv:: dependency.
//   [v2] FPU / ACCEL / other functional units are ignored: their rs1/rs2/rd name
//        FP registers, so counting them as integer registers is wrong.
//   [v2] New outputs the counter bank needs: pc_blk_o / mem_blk_o (block bitmaps,
//        lossless when two branches retire together) and retire_cnt_o.
//   [v2] mem_addr_i is now actually used (it was unused before).
//
// Semantics kept from the original:
//   * reg_inc_o[r] = register x_r was touched by at least one retiring instruction
//     THIS CYCLE (OR across ports) -> a register counts at most once per cycle,
//     even if both commit ports touch it.
//   * x0 is never touched.
//   * Masked: FP load rd, FP store rs2, CSR-immediate rs1 (field holds an immediate).
// ------------------------------------------------------------------------------
module dfdtmh_event_gen
  import ariane_pkg::*;
  import dfdtmh_pkg::*;
#(
  parameter int unsigned NR_COMMIT_PORTS    = 2,
  parameter type         scoreboard_entry_t = logic,
  parameter int unsigned MemAddrWidth       = 56,   // [v2] CVA6 PLEN
  parameter int unsigned RetW               =       // [v2] width of retire_cnt_o
      (NR_COMMIT_PORTS <= 1) ? 1 : $clog2(NR_COMMIT_PORTS + 1)
) (
  // Commit interface (same style as SBTMH / cva6_tmh_unit)
  input  scoreboard_entry_t [NR_COMMIT_PORTS-1:0] commit_instr_i,
  input  logic              [NR_COMMIT_PORTS-1:0] commit_ack_i,

  // Address of the load/store that retires this cycle (e.g. rvfi_mem_paddr).
  // Only looked at when a load/store retires, so it may be garbage otherwise.
  input  logic [MemAddrWidth-1:0]                 mem_addr_i,

  output logic [RegNum-1:0]                       reg_inc_o,    // registers touched this cycle
  output logic                                    pc_set_o,     // a branch/jump retired
  output logic [PcBlkW-1:0]                       pc_blk_o,     // [v2] PC blocks of retiring branches/jumps
  output logic                                    mem_set_o,    // a load/store retired
  output logic [MemBlkW-1:0]                      mem_blk_o,    // [v2] memory block of the retiring load/store
  output logic                                    retire_o,     // at least one instruction retired
  output logic [RetW-1:0]                         retire_cnt_o  // [v2] number of instructions retired
);

  localparam int unsigned PcIdxW  = $clog2(PcBlkW);
  localparam int unsigned MemIdxW = $clog2(MemBlkW);

  // ---------------------------------------------------------------------------
  // Internal signals
  // ---------------------------------------------------------------------------
  logic [RegNum-1:0]  reg_inc_raw [NR_COMMIT_PORTS];
  logic               pc_set_raw  [NR_COMMIT_PORTS];
  logic [PcBlkW-1:0]  pc_blk_raw  [NR_COMMIT_PORTS];
  logic               mem_set_raw [NR_COMMIT_PORTS];

  // ---------------------------------------------------------------------------
  // Per-port combinational decode
  // ---------------------------------------------------------------------------
  always_comb begin
    scoreboard_entry_t instr;
    logic              uses_int;

    instr    = '0;
    uses_int = 1'b0;

    // Default
    for (int p = 0; p < NR_COMMIT_PORTS; p++) begin
      reg_inc_raw[p] = '0;
      pc_set_raw[p]  = 1'b0;
      pc_blk_raw[p]  = '0;
      mem_set_raw[p] = 1'b0;
    end

    for (int p = 0; p < NR_COMMIT_PORTS; p++) begin
      if (commit_ack_i[p]) begin
        instr = commit_instr_i[p];

        // [v2] only these units use INTEGER rs1/rs2/rd; FPU/ACCEL fields are FP regs
        uses_int = (instr.fu inside {ALU, CTRL_FLOW, MULT, LOAD, STORE, CSR});

        // ---------------------------------------------------------------
        // 1. Register accesses (rs1, rs2, rd)
        // ---------------------------------------------------------------
        if (uses_int) begin
          // rs1
          if (instr.rs1 != 5'd0) begin
            // CSR immediate form: rs1 field holds the immediate -> ignore
            if (!(instr.fu == CSR && instr.use_zimm)) begin
              reg_inc_raw[p][instr.rs1] = 1'b1;
            end
          end

          // rs2
          if (instr.rs2 != 5'd0) begin
            // FP store: rs2 is an FP register -> ignore
            if (!(instr.fu == STORE && (instr.op inside {FSB, FSH, FSW, FSD}))) begin
              reg_inc_raw[p][instr.rs2] = 1'b1;
            end
          end

          // rd
          if (instr.rd != 5'd0) begin
            // FP load: rd is an FP register -> ignore
            if (!(instr.fu == LOAD && (instr.op inside {FLB, FLH, FLW, FLD}))) begin
              reg_inc_raw[p][instr.rd] = 1'b1;
            end
          end
        end

        // ---------------------------------------------------------------
        // 2. PC monitor - only branches / jumps
        // ---------------------------------------------------------------
        if (instr.fu == CTRL_FLOW) begin
          pc_set_raw[p]                         = 1'b1;
          pc_blk_raw[p][instr.pc[PcShift+:PcIdxW]] = 1'b1;   // [v2]
        end

        // ---------------------------------------------------------------
        // 3. Memory monitor - load or store
        // ---------------------------------------------------------------
        if (instr.fu inside {LOAD, STORE}) begin
          mem_set_raw[p] = 1'b1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Merge all ports (OR reduction)
  // ---------------------------------------------------------------------------
  always_comb begin
    reg_inc_o    = '0;
    pc_set_o     = 1'b0;
    pc_blk_o     = '0;
    mem_set_o    = 1'b0;
    mem_blk_o    = '0;
    retire_o     = 1'b0;
    retire_cnt_o = '0;

    for (int p = 0; p < NR_COMMIT_PORTS; p++) begin
      reg_inc_o |= reg_inc_raw[p];
      pc_set_o  |= pc_set_raw[p];
      pc_blk_o  |= pc_blk_raw[p];
      mem_set_o |= mem_set_raw[p];
      retire_o  |= commit_ack_i[p];
      if (commit_ack_i[p]) retire_cnt_o = retire_cnt_o + 1'b1;
    end

    // [v2] one load/store retires per cycle (single address tap)
    if (mem_set_o) mem_blk_o[mem_addr_i[MemShift+:MemIdxW]] = 1'b1;
  end

  // pragma translate_off
  initial begin
    assert (RegNum == 32) else $fatal(1, "dfdtmh_event_gen: RegNum must be 32");
    assert ((PcBlkW  & (PcBlkW  - 1)) == 0 && PcBlkW  >= 2) else $fatal(1, "PcBlkW must be a power of two >= 2");
    assert ((MemBlkW & (MemBlkW - 1)) == 0 && MemBlkW >= 2) else $fatal(1, "MemBlkW must be a power of two >= 2");
    assert (MemShift + MemIdxW <= MemAddrWidth) else $fatal(1, "memory block index exceeds MemAddrWidth");
  end
  // pragma translate_on

endmodule