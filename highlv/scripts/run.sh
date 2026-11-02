#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)
cd "$BASE"
export PYTHONPATH="$BASE/src${PYTHONPATH:+:$PYTHONPATH}"
if [ -d /oem/usr/lib ]; then
  export LD_LIBRARY_PATH="/oem/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "${RB_PYTHON:-python3}" -u -m rb_runtime "$@"
