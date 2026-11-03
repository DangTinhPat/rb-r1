#!/bin/sh
set -eu
BASE=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd -P)
case "${1:-host}" in
  host)
    cmake -S "$BASE" -B "$BASE/build/host" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BASE/build/host" --parallel 4
    ;;
  arm)
    : "${RB_CROSS_ROOT:?Set RB_CROSS_ROOT to the extracted ARM cross-toolchain root}"
    export LD_LIBRARY_PATH="$RB_CROSS_ROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    FLAGS="-B$RB_CROSS_ROOT/usr/arm-linux-gnueabihf/bin -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard"
    cmake -S "$BASE" -B "$BASE/build/arm" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=arm \
      -DCMAKE_CXX_COMPILER="$RB_CROSS_ROOT/usr/bin/arm-linux-gnueabihf-g++-13" \
      -DCMAKE_C_COMPILER="$RB_CROSS_ROOT/usr/bin/arm-linux-gnueabihf-gcc-13" \
      -DCMAKE_CXX_FLAGS="$FLAGS" -DCMAKE_C_FLAGS="$FLAGS"
    cmake --build "$BASE/build/arm" --parallel 4
    ;;
  *) echo 'usage: scripts/build.sh host|arm' >&2; exit 2 ;;
esac
