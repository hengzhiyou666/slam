#!/usr/bin/env bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/setup_env.sh"
MAP_PATH=${MAP_PATH:-$SCRIPT_DIR/maps/map.pcd}

exec ros2 launch fast_lio relocalizing.launch.py \
  map_path:="$MAP_PATH" "$@"
