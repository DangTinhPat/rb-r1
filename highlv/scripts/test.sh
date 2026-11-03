#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)
export PYTHONPATH="$BASE/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
cd "$BASE"
# Native runtime first (needs `scripts/build.sh host`); a failure stops here.
"$BASE/build/host/bin/rb_runtime_tests"
exec "${RB_PYTHON:-python3}" -m unittest discover -s tests -p 'test_*.py' -v
