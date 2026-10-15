#!/usr/bin/env bash
# Run the mission-planner pytest suite and the simulator gtests without ROS.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Prefer g++. Some images point /usr/bin/c++ at clang without a libstdc++ link.
export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"

if ! python3 -c "import pytest, yaml" >/dev/null 2>&1; then
  echo "python3-pytest and python3-yaml are required" >&2
  exit 1
fi

python3 -m pytest -q "${ROOT}/src/mission_planner"

CMAKE_ARGS=()
GTEST_CMAKE="/usr/lib/x86_64-linux-gnu/cmake/GTest/GTestConfig.cmake"
if [[ ! -f "${GTEST_CMAKE}" ]] && ! pkg-config --exists gtest 2>/dev/null; then
  if [[ ! -d /usr/src/googletest ]]; then
    echo "libgtest-dev is required to build the C++ tests" >&2
    exit 1
  fi
  cmake -S /usr/src/googletest -B /tmp/googletest-build \
    -DCMAKE_INSTALL_PREFIX=/tmp/googletest >/dev/null
  cmake --build /tmp/googletest-build -j"$(nproc)" >/dev/null
  cmake --install /tmp/googletest-build >/dev/null
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=/tmp/googletest")
fi

BUILD="${ROOT}/build/pure"
cmake -S "${ROOT}/src/drone_sim/standalone" -B "${BUILD}" "${CMAKE_ARGS[@]}"
cmake --build "${BUILD}" -j"$(nproc)"
ctest --test-dir "${BUILD}" --output-on-failure
