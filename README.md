# Secure NoC Router

A 4-port Network-on-Chip router in Verilog with **packet-level security**: an ACL that blocks forbidden source→destination paths, running-XOR **parity checking**, and **QoS-aware arbitration**. Two micro-architectures are included so they can be compared directly.

| | Phase I | Phase II |
|---|---|---|
| Structure | Central arbiter → shared parser → RCU → crossbar | Per-port FIFO → parser → RCU → 4×4 crossbar with a **per-output arbiter** → output FIFO |
| Parallelism | One packet processed at a time | All 4 input ports processed in parallel; only real output contention stalls a lane |
| Flow control | – | Registered stages with valid/ready backpressure |
| Packet ordering | – | Wormhole lock: an output stays locked to one input until the parity word passes, so packets never interleave |

Both phases expose the **same top-level interface** (`secure_noc_router_top`), so the same wrappers and testbench ports work for either.

## Parameters and packet format

- Data width 16, 4 ports, FIFO depth 8
- Ports: `0 = North, 1 = South, 2 = East, 3 = West`
- Header: `{8'h00, payload_len[3:0], dest[1:0], qos[1:0]}`
- Input packet: `HEADER → PAYLOAD[0..len-1] → PARITY` (parity is the XOR of header and all payload words)
- Output packet: `PAYLOAD[0..len-1] → PARITY` (the header is consumed by the router)
- QoS: High > Medium > Low, round-robin tie-break
- ACL (indexed by `{src, dest}`): N→W and E→S are blocked, everything else is allowed

## Repository layout

```
phase1/
  rtl/    Phase I router
  tb/     self-checking testbench
  fpga/   ZedBoard demo wrapper (VIO/ILA driven) + XDC
phase2/
  rtl/    secure_noc_router_phase2.v  (pipelined router)
  tb/     secure_noc_router_tb_phase2.v  (self-checking testbench)
```

> **Important:** both phases define a module named `secure_noc_router_top`. Never add Phase I and Phase II RTL to the same Vivado project.

## Simulating Phase II in Vivado (2022.2)

1. Create a project and add `phase2/rtl/secure_noc_router_phase2.v` as a **design source**.
2. Add `phase2/tb/secure_noc_router_tb_phase2.v` as a **simulation source** and make `secure_noc_router_tb` the simulation top.
3. Run Behavioral Simulation, then `run all` in the Tcl console (or set `xsim.simulate.runtime` to `-all`).
4. The run ends with `RESULT: ALL CHECKED TESTS PASSED` when everything passes.

The testbench has a `TESTCASE` localparam: `0` runs the full regression, `1`–`15` run a single test.

### Phase II verification status

Full regression (`TESTCASE = 0`) in Vivado XSim: **105 checks passed, 0 failed**.

Covered: normal routing, payload ordering, ACL blocking (both rules plus an exhaustive sweep of all 12 src≠dest pairs), correct and corrupted parity, QoS priority, round-robin fairness, four-port contention, zero- and maximum-length payloads, reset recovery, FIFO-full backpressure, parallel outputs to four different ports, and background assertions (per-output one-hot grants, wormhole lock, no lane granted by two outputs).

## FPGA demo (Phase I)

`phase1/fpga/` holds a ZedBoard (XC7Z020-CLG484-1) wrapper that drives the router from Vivado VIO probes and reports status on LD0–LD7, plus the matching constraints. It expects `vio_0` and `ila_0` IP cores to be generated in the Vivado project.

## Authors

<your name> (add co-authors or advisor here if you like)
