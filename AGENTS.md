# RB-R1 runtime maintenance

Read `README.md`, `docs/architecture.md`, `docs/protocol.md` and the relevant
producers/consumers before changing the control flow. This project is the
upper-layer Luckfox runtime, not MCU firmware or a simulator.

- TARGET: `[hipL,hipR,crankL,crankR,omegaL,omegaR]`, four absolute radians and
  two rad/s values. Feedback q: hipL/R,kneeL/R,wheelL/R, knee = crank - hip.
- Recovery FF: hipL/R,kneeL/R rates. Optional upstream torque is not actor input.
- Preserve trained action mapping, ONNX embedded normalization vs RKNN external
  normalization, absolute policy schedule, 50-step warm-up and GRU handoff.
- Keep recovery/support behaviour while pruning dependencies. The core still
  needs supported balance until policy owns the output. Do not replace it with
  pose playback or bypass a fault/deadline to make tests pass.
- Run `scripts/build.sh host`, `scripts/test.sh`, `scripts/smoke_test.py` for
  related changes. Mapping changes require `tests/test_flow_contract.py`.
- Build ARM when changing native/build code and the cross-toolchain is available.
  Report host, cross-build, CPU, NPU, transport and physical evidence separately.
- Do not operate physical motors, flash firmware or deploy without an explicit
  current request. Unit/mock tests must use temporary local endpoints.
- The protocol C codec is a reference; the test-only MCU link fixture is not the
  production MCU gateway. Production applied ACK requires both motor buses.
- No Git initialization or remote configuration unless requested by the user.
