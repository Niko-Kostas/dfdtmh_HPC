# dfdtmh_HPC – Design-for-Debug Tailor-Made Hardware Performance Counters

SystemVerilog implementation of a Design-for-Debug inspired set of **Hardware Performance Counters** (HPCs) for the CVA6/Ariane core.

---

### 1. File list (RTL + testbenches)

| File | Description |
|------|-------------|
| `dfdtmh_pkg.sv` | Shared parameters & types |
| `dfdtmh_event_gen.sv` | Event generation from commit ports |
| `dfdtmh_ctrl_fsm.sv` | Interval / control FSM |
| `dfdtmh_counter_bank.sv` | Counters + bitmaps |
| `dfdtmh_csr.sv` | CSR interface |
| `cva6_dfdtmh_unit.sv` | Top-level unit |
| `tb_dfdtmh_*.sv` | Self-checking testbenches |

### 2. Reproducible Simulation Flow

All testbenches are self-checking. A successful run ends with a clear **PASS** message. Any mismatch against the reference model prints a `FAIL:` line.

### Prepare the file list

```bash
cd ~/tailor_made_hpc_cva6/sim
cp files.f files_dfdtmh.f
nano files_dfdtmh.f
```
Put the following content into files_dfdtmh.f
(the last line must be the testbench you want to run):

```bash
../include/ariane_pkg_stub.sv
../include/dfdtmh_pkg.sv
../rtl/dfdtmh_event_gen.sv
../rtl/dfdtmh_counter_bank.sv
../rtl/dfdtmh_ctrl_fsm.sv
../rtl/dfdtmh_csr.sv
../rtl/cva6_dfdtmh_unit.sv
../tb/tb_dfdtmh_unit.sv          # ← change this line for other testbenches
```
### Run Simulation:
from the sim/ directory:
```bash
source tsmc65.sh
xrun -sv -access +rwc -timescale 1ns/1ps -f files_dfdtmh.f -top tb_dfdtmh_unit
```
Waveform Dumping:
To generate VCD and open SimVision:
```bash
xrun -sv -access +rwc -timescale 1ns/1ps -f files_dfdtmh.f -top tb_dfdtmh_unit +FULLDUMP
simvision cva6_dfdtmh_unit.vcd &
```

Notes:
Currently only dfdtmh_unit tb has waveform
Still in process of analyzing waveform and editing RTL code

