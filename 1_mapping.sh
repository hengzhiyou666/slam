#!/usr/bin/env bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/setup_env.sh"

# mapping.yaml 使用 ./maps/... 相对路径。
# 先进入脚本所在目录，保证地图始终保存到 2_slam/maps/。
cd "$SCRIPT_DIR"

exec ros2 launch fast_lio mapping.launch.py "$@"
