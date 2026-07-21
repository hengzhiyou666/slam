#!/usr/bin/env bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# 普通电脑通常把 ROS 安装在 /opt/ros；dog3 的厂家 ROS 位于 /app。
if [[ -f /opt/ros/humble/setup.bash ]]; then
  ROS_SETUP=/opt/ros/humble/setup.bash
  DEFAULT_ROS_DOMAIN_ID=24
elif [[ -f /app/opt/ros/humble/local_setup.bash ]]; then
  ROS_SETUP=/app/opt/ros/humble/local_setup.bash
  DEFAULT_ROS_DOMAIN_ID=0
else
  echo "错误：未找到 ROS 2 Humble 环境" >&2
  return 2 2>/dev/null || exit 2
fi

set +u
source "$ROS_SETUP"

# dog3 的自定义消息和 Zenoh/RMW 环境由厂家文件提供。
if [[ -f /app/idl_msgs/local_setup.bash ]]; then
  source /app/idl_msgs/local_setup.bash
fi
if [[ -f /app/script/env.sh ]]; then
  source /app/script/env.sh
fi

# dog3 没有全局安装 colcon、Eigen 和 PCL；部署目录中的 .deps 提供它们。
DEPS_DIR="$SCRIPT_DIR/.deps"
if [[ -d "$DEPS_DIR/venv/bin" ]]; then
  export PATH="$DEPS_DIR/venv/bin:$PATH"
fi
if [[ -d "$DEPS_DIR/venv/lib/python3.10/site-packages" ]]; then
  export PYTHONPATH="$DEPS_DIR/venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ -d "$DEPS_DIR/root/usr" ]]; then
  export CMAKE_PREFIX_PATH="$DEPS_DIR/ros_sdk_overlay:$DEPS_DIR/root/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export CPATH="$DEPS_DIR/root/usr/include${CPATH:+:$CPATH}"
  export LIBRARY_PATH="$DEPS_DIR/sdk_sysroot/usr/lib/aarch64-linux-gnu:$DEPS_DIR/root/usr/lib/aarch64-linux-gnu${LIBRARY_PATH:+:$LIBRARY_PATH}"
  export LD_LIBRARY_PATH="$DEPS_DIR/sdk_sysroot/usr/lib/aarch64-linux-gnu:$DEPS_DIR/root/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$DEFAULT_ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}

if [[ -f "$SCRIPT_DIR/install/local_setup.bash" ]]; then
  source "$SCRIPT_DIR/install/local_setup.bash"
elif [[ -f "$SCRIPT_DIR/install/setup.bash" ]]; then
  source "$SCRIPT_DIR/install/setup.bash"
fi
set -u

unset ROS_SETUP DEFAULT_ROS_DOMAIN_ID DEPS_DIR
