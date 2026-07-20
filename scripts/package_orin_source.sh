#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OUT_DIR=${OUT_DIR:-$ROOT_DIR/dist}
VERSION=${VERSION:-$(date +%Y%m%d_%H%M%S)}
MAP_PATH=${MAP_PATH:-$ROOT_DIR/maps/map.pcd}
PKG_DIR="$OUT_DIR/omni_slam_orin_source_$VERSION"
TARBALL="$OUT_DIR/omni_slam_orin_source_$VERSION.tar.gz"

mkdir -p "$OUT_DIR"
rm -rf "$PKG_DIR"
mkdir -p "$PKG_DIR"

rsync -a --delete \
  --exclude='.git' \
  --exclude='.vscode' \
  --exclude='build' \
  --exclude='build_orin' \
  --exclude='install' \
  --exclude='install_orin' \
  --exclude='log' \
  --exclude='ros_logs' \
  --exclude='dist' \
  --exclude='FAST_LIO/doc' \
  --exclude='FAST_LIO/Log' \
  --exclude='FAST_LIO/PCD' \
  --exclude='*.db3' \
  --exclude='*.bag' \
  --exclude='*.mcap' \
  --exclude='__pycache__' \
  "$ROOT_DIR/" "$PKG_DIR/"

mkdir -p "$PKG_DIR/maps"
if [[ -f "$MAP_PATH" ]]; then
  cp "$MAP_PATH" "$PKG_DIR/maps/map.pcd"
else
  echo "[package_orin_source] WARNING: map not found: $MAP_PATH" >&2
fi

cat > "$PKG_DIR/RUN_ON_ORIN.md" <<'EOS'
# Run on Orin

```bash
cd ~/omni_slam_orin_source_*
./scripts/build_native_orin.sh
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash

# Mapping
ros2 launch fast_lio mapping.launch.py

# Relocalization
ros2 launch fast_lio relocalizing.launch.py map_path:=$PWD/maps/map.pcd
```

If Livox setup is elsewhere:

```bash
export LIVOX_SETUP=/path/to/ws_livox/install/setup.bash
```
EOS

chmod +x "$PKG_DIR/scripts/"*.sh
cd "$OUT_DIR"
tar -czf "$TARBALL" "$(basename "$PKG_DIR")"
echo "$TARBALL"
