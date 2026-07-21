#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WORKERS=${WORKERS:-1}
MAKE_JOBS=${MAKE_JOBS:-2}

set +u
source "$ROOT_DIR/setup_env.sh"
set -u

EXTRA_CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE=Release
  -DUSE_LIVOX_CUSTOM_MSG=OFF
  -DUSE_LIVOX=OFF
  -DBUILD_SAC_IA_GICP=OFF
)
if [[ -d "$ROOT_DIR/.deps/root/usr" ]]; then
  EXTRA_CMAKE_ARGS+=("-DOMNI_SLAM_DEPS_ROOT=$ROOT_DIR/.deps/root")
fi

cd "$ROOT_DIR"
MAKEFLAGS="-j$MAKE_JOBS" colcon build \
  --symlink-install \
  --parallel-workers "$WORKERS" \
  --base-paths FAST_LIO icp_relocalization \
  --packages-select fast_lio icp_relocalization \
  --cmake-args "${EXTRA_CMAKE_ARGS[@]}"
