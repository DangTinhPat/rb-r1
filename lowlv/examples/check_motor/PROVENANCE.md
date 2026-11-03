# Source

Copied 2026-11-02 from local `/home/dvt/lumi-quantum/examples/check_motor`.
`UPSTREAM_SHA256.json` records SHA256 of each original file before adaptation.
No source secrets, build outputs, session logs or caches were copied.

Retained: independent ESP-IDF project, single-wire joint_bus framing module,
leg motor driver/trajectory/gains, read-only diagnostics, authenticated HTTP OTA,
OTA image/identity checking tool, golden bus test and protocol errata.

Adapted: LF-facing UART0 pins43/44 at921600, no console log on that UART;
optional final FIFO motion guard (OFF/reads bypass);
strict sequenced UART parser and QUEUED/REJECT replies; 20ms sticky lease;
OFF register readback, explicit OFF-pose MOTOR_HOME before enabling;
global fault across both workers; no stale retries, generation checks reject pre-RESET worker copies and queued enables;
wheel zero goal write/readback before torque enable; driver/active bus error latch;
Wi-Fi console made read-only; project/environment names. Added host mocked control
checks. Production HCR1 and third_party/bl4012e are untouched.

The original default full driver position range is retained; no claim of mechanical
travel calibration or hard physical 20ms shutdown latency is made.

Follow-up motor-online fix: initialize joint reply level 0x10 to 2 only after
OFF register readback, then verify level by readback. BL4012 missing OFF/ON/goal
write ACK can be recovered only by clean exact register readback; active recovery
also checks the lease/generation. Stop flooding unconfigured buses with OFF
retries every 4ms; keep the 1s configuration retry cadence. Add per-driver
configuration diagnostics, reply level and OFF readback to HTTP status.
