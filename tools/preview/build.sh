#!/usr/bin/env bash
# Build the host layout preview. Needs only a C++17 compiler and zlib -- no
# Arduino toolchain, no board, no flash.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
c++ -std=c++17 -O2 -Wall -o preview preview.cpp -lz
echo "built ./preview -- run it to emit PNGs"
