# Source

Adapted 2026-11-02 from local `/home/dvt/lumi-quantum/examples/check_motor`.
`gui_diagnostics.py` is copied unchanged from `gui/libs/gui_diagnostics.py`;
its original SHA256 is recorded in `UPSTREAM_SHA256.json`.

The Tk GUI was replaced with a standard-library LF HTTP server and browser GUI,
retaining the original six-motor mapping, wheel rpm, joint jog, incremental MOVE,
gains and validated telemetry parser. Added single GUI ownership, browser lease,
serial heartbeat/command sequencing, token access, freshness checks and mock tests.
This example has no imports/build dependency on lowlv or Lumi-Quantum.

Follow-up GUI fix: fence delayed HTTP status/error responses by session version;
Connect and STOP cannot be undone by an earlier poll. Pulse returns its snapshot
in one HTTP request. Added explicit Online vs HOME counts, ESP ACK and STOP
reason, counters and private local diagnostics/token files. Node VM regression
tests execute the actual inline script with mocked DOM/fetch (no motor endpoint).

LF timing follow-up: schedule UART thread SCHED_FIFO20, reduce Python GIL switch
interval to 1ms; HTTP/1.1 persistent connections and TCP_NODELAY; explicit Connect
separate from lease renewal, so an expired pulse cannot silently reclaim a GUI
session. No change to the MCU 20ms lease or production HCR1 contract.

Further stability fix supersedes the shared-thread/polling design above:
UART interpreter moved into its own realtime process with bounded Unix
seqpacket RPC and fail-closed EOF handling; browser heartbeat moved into a
dedicated Web Worker and authenticated full-duplex WebSocket. Fixed50ms cadence
independent of HTTP response/UI latency. Each Connect uses a fresh client ID;
old streams cannot disarm a different new owner. No timeout increases. Added
process EOF/GIL-stall tests, websocket mask/auth tests and worker delayed-pong
tests. This example still has no lowlv code/import/build dependency.
