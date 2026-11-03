#!/bin/sh
# Host tests. Needs `scripts/build.sh host` first: they load the native libraries.
set -eu
BASE=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)
export PYTHONPATH="$BASE/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
cd "$BASE"
exec "${RB_PYTHON:-python3}" -m unittest discover -s tests -p 'test_*.py' -v
