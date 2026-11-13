# dfdtmh_HPC - Design for Debug Tailor Made Hardware Performance Counters (dfdtmh HPC's)

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

### 3. Running Verilator

Running Script:

```bash
cat > sim/run_dfdtmh.sh <<'SCRIPT'
#!/usr/bin/env bash
# usage: bash run_dfdtmh.sh tb_dfdtmh_event_gen
cd "$(dirname "$0")"
source ./tsmc65.sh
tb=$1
cat > /tmp/files_$tb.f <<EOF
../include/ariane_pkg_stub.sv
../include/dfdtmh_pkg.sv
../rtl/dfdtmh_event_gen.sv
../rtl/dfdtmh_counter_bank.sv
../rtl/dfdtmh_ctrl_fsm.sv
../rtl/dfdtmh_csr.sv
../rtl/cva6_dfdtmh_unit.sv
../tb/$tb.sv
EOF
xrun -sv -access +rwc -timescale 1ns/1ps -f /tmp/files_$tb.f -top $tb
SCRIPT
