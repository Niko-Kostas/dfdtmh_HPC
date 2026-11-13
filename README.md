# dfdtmh_HPC - Design for Debug Tailor Made Hardware Performance Counters (dfdtmh HPC's)
Design for debug - tailor made HPC project.


(or the equivalent message printed by the individual testbench). Any mismatch against the reference model prints a `FAIL:` line and increments an error counter.

### 1. Prerequisites

- A SystemVerilog simulator that supports IEEE 1800 (tested / recommended):
  - **Verilator** (open-source, recommended for quick checks)
  - Synopsys VCS, Cadence Xcelium, or Siemens Questa
- The files in this repository (no external IP is required for the unit-level testbenches).

> Note: `tb_dfdtmh_unit.sv` imports `ariane_pkg::*`. A minimal stub (`ariane_pkg_stub.sv`) is provided so the testbench can be compiled standalone.

### 2. File list (RTL + testbenches)

| File | Description |
|------|-------------|
| `dfdtmh_pkg.sv` | Shared parameters & types |
| `dfdtmh_event_gen.sv` | Event generation from commit ports |
| `dfdtmh_ctrl_fsm.sv` | Interval / control FSM |
| `dfdtmh_counter_bank.sv` | Counters + bitmaps |
| `dfdtmh_csr.sv` | CSR interface |
| `cva6_dfdtmh_unit.sv` | Top-level unit |
| `tb_dfdtmh_*.sv` | Self-checking testbenches |

### 3. Running with Verilator (example)

```bash
# Example for the top-level unit testbench
verilator --binary -Wall --timing \
  --top-module tb_dfdtmh_unit \
  ariane_pkg_stub.sv \
  dfdtmh_pkg.sv \
  dfdtmh_event_gen.sv \
  dfdtmh_ctrl_fsm.sv \
  dfdtmh_counter_bank.sv \
  dfdtmh_csr.sv \
  cva6_dfdtmh_unit.sv \
  tb_dfdtmh_unit.sv

./obj_dir/Vtb_dfdtmh_unit
