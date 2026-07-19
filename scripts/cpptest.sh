#!/usr/bin/env bash
# Build and run the native C++ tests (doctest + CTest). See /testing.md.
#
# Usage:
#   scripts/cpptest.sh                 # build + run all tests
#   scripts/cpptest.sh -R lb           # ctest filter: only tests matching "lb"
#   BUILD_DIR=/tmp/rg scripts/cpptest.sh
#
# Any extra args are forwarded to ctest.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/cpp/build-tests}"

# Configure once. CMake auto-reconfigures on CMakeLists.txt changes at build time,
# so this only runs on a fresh build dir.
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cmake -S "$REPO_ROOT/cpp" -B "$BUILD_DIR" -DROADGEOMETRY_BUILD_TESTS=ON
fi

cmake --build "$BUILD_DIR" -j
ctest --test-dir "$BUILD_DIR" --output-on-failure "$@"
