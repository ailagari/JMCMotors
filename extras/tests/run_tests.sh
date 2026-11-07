#!/bin/bash
# Host tests for JMCMotors: the library sources against simulated drives.
# Needs a C++ compiler: g++ / clang++, or "pip install ziglang" (zig c++).
# Usage: bash extras/tests/run_tests.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../../src"
OUT="${TMPDIR:-/tmp}/jmc_tests"
mkdir -p "$OUT"
if command -v g++ >/dev/null 2>&1; then CXX="g++"
elif command -v clang++ >/dev/null 2>&1; then CXX="clang++"
else CXX="py -m ziglang c++"; command -v py >/dev/null 2>&1 || CXX="python3 -m ziglang c++"
fi
FLAGS="-std=c++17 -O1 -Wall -Wno-unused-function -I$HERE/stub -I$SRC -I$HERE"
FILES="$HERE/test_main.cpp $SRC/JMCBus.cpp $SRC/JMCMotor.cpp $SRC/JMCController.cpp"

echo "== full profile (CONTROLLINO / Mega / ESP32)"
$CXX $FLAGS $FILES -o "$OUT/full.exe"
"$OUT/full.exe"

echo "== frame check: every command's bus frames against the JMC manual"
$CXX $FLAGS "$HERE/test_frames.cpp" $SRC/JMCBus.cpp $SRC/JMCMotor.cpp $SRC/JMCController.cpp -o "$OUT/frames.exe"
"$OUT/frames.exe"

echo "== motion check: forward/reverse, several motors, homing 1-12 on a simulated axis, limits, status, resets"
$CXX $FLAGS "$HERE/test_motion.cpp" $SRC/JMCBus.cpp $SRC/JMCMotor.cpp $SRC/JMCController.cpp -o "$OUT/motion.exe"
"$OUT/motion.exe"

echo "== Uno profile"
$CXX $FLAGS -DSMALL_PROFILE -DJMC_MAX_MOTORS=6 -DJMC_CMD_BUFFER=96 -DJMC_REPLY_BUFFER=128 \
  -DJMC_FULL_REPORTS=0 -DJMC_FEATURE_SYNC=0 -DJMC_FEATURE_SAVE=0 \
  -DJMC_FEATURE_EXTRA_STATUS=0 -DJMC_FEATURE_HINTS=0 -DJMC_FEATURE_EXTRAS=0 $FILES -o "$OUT/uno.exe"
"$OUT/uno.exe"
