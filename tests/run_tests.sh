#!/usr/bin/env bash
# Builds and runs the host tests with AddressSanitizer and UBSan.
# Environment: Debian/WSL, Linux or macOS with g++ or clang++.
#
# Usage: tests/run_tests.sh [fuzz_scale]
#   fuzz_scale multiplies the number of fuzz iterations (default 1, a few seconds).
set -euo pipefail

cd "$(dirname "$0")"
CXX="${CXX:-g++}"
SCALE="${1:-1}"
SRC=../components/oregon_v21_receiver
OUT=build
mkdir -p "$OUT"

FLAGS=(-std=c++17 -O1 -g -Wall -Wextra -Werror -fno-omit-frame-pointer
       -fsanitize=address,undefined -fno-sanitize-recover=all -I"$SRC")

"$CXX" "${FLAGS[@]}" test_decoder.cpp "$SRC/manchester_decoder.cpp" "$SRC/oregon_protocol.cpp" -o "$OUT/test_decoder"
"$CXX" "${FLAGS[@]}" fuzz_decoder.cpp "$SRC/manchester_decoder.cpp" "$SRC/oregon_protocol.cpp" -o "$OUT/fuzz_decoder"
# The ESPHome glue is built against minimal header stubs in tests/stubs.
"$CXX" "${FLAGS[@]}" -Istubs test_component.cpp "$SRC/oregon_v21_receiver.cpp" "$SRC/manchester_decoder.cpp" \
  "$SRC/oregon_protocol.cpp" "$SRC/rssi_tracker.cpp" -o "$OUT/test_component"

echo "== unit tests"
"$OUT/test_decoder" data/rxb6_raw_frames.txt
echo "== component tests"
"$OUT/test_component" data/rxb6_raw_frames.txt
echo "== fuzz tests"
"$OUT/fuzz_decoder" data/rxb6_raw_frames.txt "$SCALE"
