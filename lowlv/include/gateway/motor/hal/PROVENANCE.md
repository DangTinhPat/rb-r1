# Motor HAL imported from Lumi-Quantum — 2026-11-02

Source: `/home/dvt/lumi-quantum`, HEAD
`7b86053c89620484a3c99e869cce3c23dd39d37f`.
`SOURCE_SHA256.json` records the actual source file hashes (including any local
source edits). No license is inferred or added. All imported code lives in
lowlv; building it needs neither Lumi-Quantum nor highlv.

- `MotorFrameCodec.hpp`: copied from `firmware/hal/include/rb1/hal/`.
  Namespace changed to `gateway::hal`; local bounds/null checks reject malformed
  wheel LEN (including underflow and trailing data), oversized/null writes and
  oversized/null fake replies. Valid frames retain the source bytes.
- `MotorMapping.hpp`: same source directory; namespace changed only. Driver IDs,
  mirrored signs and knee coupling are the Lumi mapping. Source joint indices
  `[LH,LK,LW,RH,RK,RW]` are metadata, not HCR ordering.
- `tests/lumi_motor_frame_tests.cpp`: adapted source golden codec tests, with
  local malformed-input checks, run by `tests/check_bl_motor.py` under ASan/UBSan.
- `BlMotorConfig`: defaults from `BoardConfig.hpp`: hip Kp10/Kd.036,
  knee Kp20/Kd.05 for each HCR profile. Unlike source gain clamping, HCR rejects
  unsupported gains/targets explicitly. Driver limits still come from READ0x32.
- `EspMotorBus`/`BlMotorWorker`: adapt the source `Esp32Bus.cpp` wheel response
  level detection and no-write-reply exchange (exact echo +120us settle).
  The guarded C transport remains responsible for FIFO TX, one-wire pad release,
  turnaround and late-drain; its verified upstream remains unchanged. The live
  ESP adapter validates returned frames with the imported codec as well.

HCR adaptation: physical driver ID3 measures knee directly in Lumi, so default
`coupling=0`. Define `d=coupling*hip+knee`; convert HCR `phi_ref` to
`d_ref=phi_ref+(coupling-1)*hip_ref`, and FF to
`d_dot_ff=knee_dot_ff+coupling*hip_dot_ff`. Home and feedback use the same
coordinate. Convert quantized d/hip back to virtual phi in applied_target.
`coupling=1` remains available explicitly for a physical crank encoder. HCR frame,
layout, profiles, flags, TTL20ms and source-age30ms are unchanged.

The source asynchronous `MotorTransport` queue is not used as an applied ACK:
HCR retains cancellation, fault latch, verified OFF, HOME after boot, and requires
both bus workers to complete the same batch. Even the wheel echo-only path must
READ the target/enable register before success; bad echo/CRC/status is a fault.
Joint response level2 and MIT mode3 are verified by readback while OFF.
Unsupported wheel response-level reads fault during disarmed setup.

Source controller-side knee trim/integrator and knee lead are not motor HAL and
are not imported. The source recovery trajectory is handled in highlv separately.

Verification on laptop: gateway host checks, motor backend tests and 58 codec
checks, inherited C + ESP adapter SDK mocks, 12 Python tool tests, and PlatformIO
`esp32-s3-hybrid-mcu` build passed. SDK mocks include the no-write-reply path,
corrupted/missing echo, deadline/cancellation during settle, and negative controls.
Firmware size: RAM61864 bytes, flash1004391 bytes. The build's optional IDF size
report warns about unsupported `--ng`; normal size check and binary generation
succeed. No OTA/flash/deploy, electrical timing measurement or physical motor
operation was performed. Lumi's physical evidence belongs to the source revision;
it is not evidence that this adapted firmware has already run on the robot.
