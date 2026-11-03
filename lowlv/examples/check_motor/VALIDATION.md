# Validation 2026-11-02

- **Host control:** `tests/check_control.py` passed, actual UART parser, lease and
  leg workers compiled with mocked ESP-IDF/buses under ASan/UBSan. Includes exact
  20ms expiry, sticky expiry, stale sequence, malformed commands, HOME prerequisite,
  lost enable ACK, zero-speed readback, driver fault, stale pre-RESET worker copy
  and no expired MIT retry.
- **Host bus:** `tests/check_motor_bus.py` passed, actual single-wire bus codec,
  golden frames, checksums, echo/reply failures, late replies, restored sync and
  final FIFO guard rejecting motion while permitting OFF.
- **Host transport/API:** four tests in
  `highlv/examples/check_motor/tests/test_server.py` passed. Temporary PTY and
  authenticated localhost HTTP only; command delivery, ACK reception, telemetry
  validation, GUI ownership and stale-link STOP. No real UART opened by tests.
- **ESP cross-build:** both `check-motor` and `check-motor-ota` passed with
  PlatformIO espressif32@6.12.0. OTA credentials are local/ignored.
- **Existing highlv host/CPU:** `scripts/build.sh host`, `scripts/test.sh`
  (27 tests) and `scripts/smoke_test.py` passed. Smoke uses ONNX and temporary
  Unix sockets, physical_hardware=false. Production source unchanged.
- **LF execution:** copied GUI to `/userdata/highlv/examples/check_motor` on
  root@10.10.60.232; Python3.11.6 ran `server.py --help`, including imports.
  Did not start its UART worker or HTTP listener on LF.
- **Browser:** inline JavaScript passed `node --check`; local HTTP routes tested.
  Visual/browser interaction not verified: no browser surface available in session.
- **NPU:** not exercised; this independent bench has no inference.
- **Physical:** no firmware upload and no motor operation in this change.
  The earlier physical hello/im-here test applies only to check_hcr_link, not
  this new motor firmware. No measured motor shutdown latency or mechanical
  calibration evidence is claimed.

## Physical OTA / UART verification (subsequent user request, 2026-11-02)

- User explicitly requested OTA upload of this motor test firmware. Used the
  earlier user-confirmed PCB-only/no-motors condition; no motor ON, HOME or RESET
  command was sent.
- Previous firmware at 10.10.61.85 identified itself as the Arduino UART hello
  check. Arduino OTA transfer returned 0 (success), application image only.
- Firmware image: 876608 bytes; SHA256
  `f127c31df73c05cd6de2fecdb45afcb45ff36e4dc5915bdbc97f5536b7847f9a`.
- After reboot, authenticated HTTP status verified firmware `rb-r1-check-motor`,
  project `rb_r1_check_motor`, MAC `70:04:1d:a8:ca:fc`, running partition `app0`,
  ELF SHA256 `d7fcbd25130a7caa264fafe16235507ada2ada189d33af78b41c7e11f24d2a83`.
  All six worker enabled flags were false; all six drivers were offline.
  `off_confirmed=false`: there are no driver OFF readbacks; do not report this
  as physically verified six-motor torque OFF.
- LF old runtime PID545 (`/root/rb-r1/contoller hybrid/deploy/hybrid_runtime`)
  occupied /dev/ttyS2; stopped it with SIGTERM. Startup configuration unchanged.
- Physical LF UART @921600: sent STOP, HELLO and C1 GG only. Received
  `#HELLO MOTOR_BENCH_1`, `#A 1 QUEUED`, 10 telemetry frames and `#S 1 0 0`;
  all motor enabled flags zero. Sent final STOP and closed UART for GUI use.
- Local ignored `.local/ota.json` now contains target identity/access key for
  the subsequent bench HTTP OTA tool. No credential is included in this report.
- This verifies physical OTA, reboot identity and bidirectional LF/ESP UART.
  Motor movement, driver communication, physical shutdown timing and NPU remain
  untested. The earlier no-upload status above describes the implementation stage.

## Follow-up: connected motor drivers online (2026-11-02)

User confirmed both motor buses connected and powered. Physical diagnostics
showed the wheels responded but their missing OFF-write ACK prevented initial
configuration, and all four joints had response level 1 while the example
required 2. Added diagnostic firmware confirmed these conditions.

Final fix configures joint response level 2 while OFF and reads it back; accepts
BL4012 write-ACK absence only with matching clean torque/goal readback, preserving
active lease/generation checks. Configuration failures now expose a reason.
Tests check actual parser/worker code with mocked reply-level setup, missing
wheel ACK recovery and failed readback rejection, plus the original bus tests.
Both firmware environments built successfully; ASan/UBSan control/bus tests passed.

Two bench HTTP OTA updates completed; each was gated by all six physical torque
OFF register readbacks. Final firmware ELF SHA256:
`b21bb94b04ed05c3a6336055e0000500e3fd0fc835ae48c764eac0371a54118e`,
MAC 70:04:1d:a8:ca:fc, partition app0. All six online=true, enabled=false,
off_confirmed=true; all four joints response_level=2; configuration errors empty.
No motor ON, jog, MOVE or wheel speed command was sent by the agent. This proves
driver discovery/configuration and OFF readbacks; movement remains untested.

## LF GUI command / timing diagnosis and fix (2026-11-02)

All six drivers remained online with zero missed feedback polls. Initial LF
standalone Bridge test received GG/RESET/MOTOR_HOME ACKs and maintained HOME
mask63 for12s, maximum UART heartbeat gap10.358ms, no gap>=20ms, no motor ON.
The remaining failures were in the LF/browser control path:

- An older GET state response could revoke a newer Connect (GUI async race).
  Added session-version fencing and explicit Connect vs pulse renewal. Expired
  heartbeat cannot silently reclaim ownership. Actual inline GUI JS regression
  tests run in Node VM with mocked DOM/fetch; delayed response and STOP cases pass.
- Physical HTTP test exposed a26.75ms UART gap under ordinary HTTP threads,
  causing the20ms MCU lease to latch correctly. UART thread now SCHED_FIFO20,
  Python GIL switch interval1ms. No MCU timing changes.
- Extra viewer HTTP load then expired the300ms GUI lease while UART max gap
  stayed11.47ms. HTTP now uses1.1 keep-alive and TCP_NODELAY, with one pulse/state
  request. GUI timeout remains300ms; MCU lease remains20ms.

Final physical test used real HTTP from laptop to LF, persistent connections,
plus two extra read-only viewers; GG/RESET/MOTOR_HOME all received QUEUED ACK.
40s /990HTTP requests,398control samples; HOME reached63 and stayed63 after
initial capture, online6, fault=false, all motor ON flags=false. Maximum UART
heartbeat gap16.75ms; max HTTP RTT230.68ms. Sent final STOP, leaving HOME0 and
all motors OFF, server still running for user. This is a40s observation, not a
hard real-time or motor-motion validation. No firmware upload in this follow-up.

Four Python mock/API tests, actual inline GUI Node regression, existing27
highlv host tests, host build and ONNX temporary-socket smoke passed. No NPU or
motor motion tested. GUI shows Online count separately from HOME, ESP ACK, stop
reason; private LF diagnostics at /tmp/rb-check-motor-status.json.
