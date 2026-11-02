#!/bin/sh
# Buildroot service wrapper; run.sh resolves all files from this checkout.
set -eu
BASE=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)
PID=${RB_PID_FILE:-/var/run/rb-runtime.pid}
LOG=${RB_LOG_FILE:-/tmp/rb-runtime.log}
STATUS=${RB_STATUS_FILE:-/tmp/rb-hybrid-status.json}
running() {
  [ -f "$PID" ] || return 1
  SERVICE_PID=$(cat "$PID")
  case "$SERVICE_PID" in ''|*[!0-9]*) return 1 ;; esac
  kill -0 "$SERVICE_PID" 2>/dev/null || return 1
  [ "$(readlink "/proc/$SERVICE_PID/cwd")" = "$BASE" ]
}
case "${1:-status}" in
  start)
    if running; then echo 'rb-runtime already running'; exit 0; fi
    shift
    nohup "$BASE/scripts/run.sh" --status-file "$STATUS" "$@" >> "$LOG" 2>&1 &
    echo "$!" > "$PID"
    sleep 1
    if ! running; then tail -20 "$LOG" >&2; exit 1; fi
    echo "rb-runtime started; status: $STATUS"
    ;;
  stop)
    if running; then
      kill "$SERVICE_PID" 2>/dev/null || true
      for attempt in 1 2 3; do running || break; sleep 1; done
      if running; then echo 'rb-runtime has not stopped yet' >&2; exit 1; fi
    fi
    rm -f "$PID"
    ;;
  status)
    if running; then cat "$STATUS"; else echo 'rb-runtime stopped'; exit 1; fi
    ;;
  *) echo 'usage: scripts/service.sh start [runtime options]|stop|status' >&2; exit 2 ;;
esac
