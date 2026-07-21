#!/usr/bin/env bash
# Build and run the native C++ tests (doctest + CTest). See /testing.md.
#
# Runs the GREEN suite (roadgeometry_tests) as the fast loop, then reports the
# NORTH-STAR acceptance target (roadgeometry_northstar) we're driving to green —
# its status is shown but never fails the loop, so a still-RED north-star (even a
# compile-RED one) stays visible without blocking the green run.
#
# Usage:
#   scripts/cpptest.sh                 # build + run the green suite, report north-star
#   scripts/cpptest.sh -R lb           # ctest filter on the green suite
#   BUILD_DIR=/tmp/rg scripts/cpptest.sh
#
# Extra args are forwarded to the green-suite ctest run.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$REPO_ROOT/cpp/build-tests}"

# Configure once. CMake auto-reconfigures on CMakeLists.txt changes at build time.
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cmake -S "$REPO_ROOT/cpp" -B "$BUILD_DIR" -DROADGEOMETRY_BUILD_TESTS=ON
fi

# --- Green suite: the fast loop. A failure here fails the script. ---
cmake --build "$BUILD_DIR" --target roadgeometry_tests -j
ctest --test-dir "$BUILD_DIR" -R roadgeometry_tests --output-on-failure "$@"

# --- North-star: report progress toward green, never fatal. ---
echo
echo "--- north-star (acceptance target we're driving to green) ---"
if cmake --build "$BUILD_DIR" --target roadgeometry_northstar -j >/dev/null 2>&1; then
    if "$BUILD_DIR/tests/roadgeometry_northstar" >/dev/null 2>&1; then
        echo "north-star: GREEN — the acceptance test passes 🎉"
    else
        echo "north-star: RED (compiles, assertions fail) —"
        echo "  run: $BUILD_DIR/tests/roadgeometry_northstar"
    fi
else
    echo "north-star: RED (does not compile yet) — the target we're building toward"
fi
