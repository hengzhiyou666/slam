#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
INSTALL_BASE=${ORIN_INSTALL_BASE:-$ROOT_DIR/install_orin}
MAP_PATH=${MAP_PATH:-$ROOT_DIR/maps/map.pcd}
OUT_DIR=${OUT_DIR:-$ROOT_DIR/dist}
VERSION=${VERSION:-$(date +%Y%m%d_%H%M%S)}
PKG_DIR="$OUT_DIR/omni_slam_orin_$VERSION"
TARBALL="$OUT_DIR/omni_slam_orin_$VERSION.tar.gz"

die() { echo "[package_orin_runtime] ERROR: $*" >&2; exit 1; }
[[ -d "$INSTALL_BASE" ]] || die "missing install tree: $INSTALL_BASE; run scripts/build_orin_cross.sh first or set ORIN_INSTALL_BASE"
[[ -f "$INSTALL_BASE/lib/fast_lio/fastlio_mapping" ]] || die "missing fast_lio executable in install tree"
[[ -f "$INSTALL_BASE/lib/icp_relocalization/icp_node" ]] || die "missing icp_node executable in install tree"
[[ -f "$MAP_PATH" ]] || die "missing map: $MAP_PATH"

rm -rf "$PKG_DIR"
mkdir -p "$PKG_DIR/maps" "$PKG_DIR/scripts"
cp -a "$INSTALL_BASE" "$PKG_DIR/install"
cp "$MAP_PATH" "$PKG_DIR/maps/map.pcd"

cat > "$PKG_DIR/setup_env.sh" <<'EOS'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROS_DISTRO_TARGET=${ROS_DISTRO_TARGET:-humble}
LIVOX_SETUP=${LIVOX_SETUP:-$HOME/ws_livox/install/setup.bash}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}
set +u
source "/opt/ros/$ROS_DISTRO_TARGET/setup.bash"
set -u
if [[ -f "$LIVOX_SETUP" ]]; then
  set +u
  source "$LIVOX_SETUP"
  set -u
else
  echo "[setup_env] WARNING: Livox setup not found: $LIVOX_SETUP" >&2
fi
set +u
source "$SCRIPT_DIR/install/setup.bash"
set -u
EOS

cat > "$PKG_DIR/1_mapping.sh" <<'EOS'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/setup_env.sh"
cd "$SCRIPT_DIR"
exec ros2 launch fast_lio mapping.launch.py "$@"
EOS

cat > "$PKG_DIR/2_relocalizing.sh" <<'EOS'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/setup_env.sh"
MAP_PATH=${MAP_PATH:-$SCRIPT_DIR/maps/map.pcd}
exec ros2 launch fast_lio relocalizing.launch.py map_path:="$MAP_PATH" "$@"
EOS

cat > "$PKG_DIR/README_RUNTIME.md" <<'EOS'
# Omni SLAM Orin Runtime Package

On the Orin board:

```bash
tar -xzf omni_slam_orin_*.tar.gz -C ~/
cd ~/omni_slam_orin_*
./1_mapping.sh
./2_relocalizing.sh
```

If Livox driver setup is not at `~/ws_livox/install/setup.bash`, set:

```bash
export LIVOX_SETUP=/path/to/ws_livox/install/setup.bash
```

Default map:

```bash
maps/map.pcd
```
EOS

chmod +x "$PKG_DIR"/*.sh
cd "$OUT_DIR"
tar -czf "$TARBALL" "$(basename "$PKG_DIR")"

echo "$TARBALL"
