# RB-R1 maintenance

One repository, two layers joined by the HCR1 UART protocol:

- `highlv/` — upper-layer Luckfox runtime (fold-down planner, policy inference,
  HCR1 master). Not MCU firmware or a simulator.
- `lowlv/` — lower-layer ESP32-S3 firmware (formerly `gatewayLF`): HCR1 slave,
  IMU, BL4012 motor buses, deadline/fault latch, OTA.

The layers are separate worlds on different processors (`highlv` on a Linux SBC,
`lowlv` on an ESP32-S3 MCU). They share no code, includes, imports or build; each
keeps its own HCR1 codec and its own emulator of the other side. Their only
meeting point is the HCR1 contract. Do not make one layer depend on the other.
See the root `README.md`.

Paths below are relative to the layer named in the heading. Read that layer's
`README.md` and docs, plus the relevant producers/consumers, before changing
control flow.

## Both layers

- TARGET: `[hipL,hipR,crankL,crankR,omegaL,omegaR]`, four absolute radians and
  two rad/s values. Feedback q: hipL/R,kneeL/R,wheelL/R, knee = crank - hip.
  No torque command crosses HCR1.
- Any change to the HCR1 frame, payload layout, flags, profiles, fault codes or
  timing (TTL 20 ms, source age 30 ms) must be applied to both layers in the
  same change: `highlv/docs/protocol.md`, `highlv/src/rb_runtime/protocol.py`,
  `highlv/protocol/include/hcr_protocol.h`, `lowlv/docs/HCR1_PROTOCOL.md`,
  `lowlv/include/gateway/protocol/hcr_protocol.h`, `lowlv/tools/hcr_protocol.py`.
  The two C codecs are currently not byte-identical; do not assume they match.
- Do not operate physical motors, flash firmware, OTA-upload or deploy without
  an explicit current request. Unit/mock tests must use temporary local
  endpoints.
- Production applied ACK requires both motor buses. The test-only MCU link
  fixture in `highlv/tests/fixtures/` is not the production gateway.
- Report host, cross-build, CPU, NPU, transport and physical evidence separately.
- Never commit Wi-Fi/OTA secrets (`lowlv/include/gateway/secrets/*Secrets.hpp`).
- No Git remote configuration unless requested by the user.

## highlv/ (Luckfox runtime)

- Fold FF: hipL/R,kneeL/R rates. Optional upstream torque is not actor input.
- Preserve the action mapping and observation layout in `policy.py` until the
  model contract is confirmed (RKNN external normalization), the absolute policy
  schedule, 50-step warm-up and GRU handoff. The shipped model has no ONNX source.
- LF plans only the fold-down (`fold.py` over `native/fold_planner.cpp`); standing
  up belongs to the policy. The old stand-up core (Kneeling/Catching/Support, LQR)
  was deleted on purpose. Do not bypass a fault/deadline to make tests pass.
- Python orchestrates; C++ only where it measurably wins. A native call through
  ctypes costs ~1-4 us, so a piece that is cheaper than that stays in Python.
- Run `scripts/build.sh host`, `scripts/test.sh`, `scripts/smoke_test.py` for
  related changes. Mapping changes require `tests/test_flow_contract.py`.
- Build ARM when changing native/build code and the cross-toolchain is available.
- The protocol C codec is a reference for interop with `lowlv/`.

## lowlv/ (ESP32-S3 firmware)

- PlatformIO env `esp32-s3-hybrid-mcu`. Host tests need no hardware:
  `tests/check_gateway.py`, `tests/check_bl_motor.py`,
  `tests/check_inherited_bus.py`, `tests/test_*.py`. Some `docs/*VALIDATION.md`
  commands still point at a removed `contoller hybrid` runtime path; do not
  treat those as current.
- `MOTOR_HOME` must run after every boot before ARM is allowed. Loss of LF,
  expired command or driver fault must leave motors off; never weaken the 20 ms
  TTL, ACK or fault-latch behaviour to make a test pass.
- `third_party/bl4012e/upstream/` is the verified original with SHA256; edit
  only `third_party/bl4012e/src` and `include`, and update `PROVENANCE.md`.
- Build, flash and OTA are physical actions: build is fine, flash/OTA needs an
  explicit current request and confirmed motor OFF.
