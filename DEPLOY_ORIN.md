# Nvidia Orin Deployment Guide

This repository contains a ROS 2 Fast-LIO2 mapping/localization pipeline and an ICP-based relocalization helper for an Omni Dog platform using Livox PointCloud2 data.

The current verified bag is:

```bash
/home/user/Downloads/rosbag2_2019_09_27-14_51_10
```

The current verified runtime configuration is:

```bash
FAST_LIO/config/mapping.yaml
FAST_LIO/config/relocalizing.yaml
FAST_LIO/launch/mapping.launch.py
FAST_LIO/launch/relocalizing.launch.py
```

## 1. Recommended Deployment Strategy

For Nvidia Orin, prefer native build on the Orin board unless build time is unacceptable.

Native build is safer because ROS 2, PCL, Eigen, Livox driver, CUDA/JetPack system libraries, and DDS shared libraries must all match the target filesystem. Cross compilation is possible, but it needs a complete Orin sysroot and more careful dependency management.

Recommended order:

1. Develop and test on x86 with rosbag.
2. Commit source, configs, and launch files only.
3. Build natively on Orin.
4. Package the installed workspace or run from the workspace.
5. Only use cross compilation after the native path is stable.

## 2. Target Board Assumptions

Recommended target:

```text
Board: Nvidia Jetson Orin / Orin NX
OS: Ubuntu 22.04 from JetPack 6.x
ROS: ROS 2 Humble
Architecture: aarch64
```

If the board is JetPack 5.x / Ubuntu 20.04, use ROS 2 Foxy instead and rebuild all dependencies consistently.

Check on the board:

```bash
uname -m
lsb_release -a
echo $ROS_DISTRO
```

## 3. Runtime Dependencies

Install ROS and system dependencies on Orin:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions \
  ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-std-srvs \
  ros-humble-visualization-msgs ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-rviz2 \
  libeigen3-dev libpcl-dev libomp-dev
```

Livox dependency:

```bash
source /home/user/ws_livox/install/setup.bash
```

On the Orin board, replace that path with the actual Livox driver workspace, for example:

```bash
source ~/ws_livox/install/setup.bash
```

The package currently depends on `livox_ros_driver2`, even though the verified bag uses `sensor_msgs/msg/PointCloud2`. Keep the driver installed and sourced.

## 4. Files To Commit

Commit source and configuration:

```text
FAST_LIO/
icp_relocalization/
scripts/
1_mapping.sh
2_relocalizing.sh
setup_env.sh
README.md
DEPLOY_ORIN.md
.gitignore
```

Do not commit generated files:

```text
build/
install/
log/
ros_logs/
*.pcd
*.db3
*.bag
```

Maps should usually be distributed as release/runtime artifacts, not as normal source commits, unless the map is small and intentionally versioned.

## 5. Native Build On Orin

Clone the repo on Orin:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone <your_repo_url> omni_slam
cd ~/robot_ws
```

Source ROS and Livox:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
```

Install dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If the board has limited memory, build with fewer workers:

```bash
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 6. Mapping On Orin

Use the current verified config:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py
```

Expected input topics:

```text
/front_lidar      sensor_msgs/msg/PointCloud2
/front_lidar/imu  sensor_msgs/msg/Imu
```

The PointCloud2 layout must contain:

```text
x y z intensity tag line timestamp
```

Important config values:

```yaml
common.lid_topic: "/front_lidar"
common.imu_topic: "/front_lidar/imu"
common.sensor_frame_id: "livox_frame"
preprocess.lidar_type: 5
preprocess.scan_line: 4
preprocess.timestamp_unit: 3
mapping.extrinsic_T: [0.0, 0.0, 0.0]
mapping.extrinsic_R: identity
```

Map saving is enabled in:

```bash
FAST_LIO/config/mapping.yaml
```

FAST-LIO saves the final map when the node exits cleanly:

```bash
Ctrl-C
```

Output:

```bash
maps/map.pcd
```

## 7. Mapping With Rosbag And RViz

For offline verification:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ws_livox/install/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ws_livox/install/setup.bash
source install/setup.bash

rviz2 -d FAST_LIO/rviz/loam_livox.rviz
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ws_livox/install/setup.bash
source install/setup.bash

ros2 bag play /home/user/Downloads/rosbag2_2019_09_27-14_51_10
```

Watch these RViz topics:

```text
/cloud_registered
/cloud_registered_body
/Laser_map
/path
/state_estimation
/tf
```

## 8. Relocalization

Relocalization uses ICP to compute an initial pose, then Fast-LIO runs in prior-map mode.

Launch:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch fast_lio relocalizing.launch.py \
  map_path:=/absolute/path/to/map.pcd \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

The same map path is passed to:

```text
icp_relocalization/icp_node map_path
fast_lio prior_map_path
```

Expected behavior:

```text
icp_node loads the map
icp_node publishes /icp_result after stable convergence
transform_publisher publishes map -> odom
fastlio_mapping receives /icp_result
fastlio_mapping initializes prior map localization
```

Useful topics:

```text
/prior_map
/transformed_cloud
/icp_result
/state_estimation
/cloud_registered
/cloud_registered_body
/tf
```

## 9. Packaging For Orin

After a native build on Orin, the simplest package is the workspace source plus map:

```bash
cd ~/robot_ws/src
tar --exclude='omni_slam/build' \
    --exclude='omni_slam/install' \
    --exclude='omni_slam/log' \
    --exclude='omni_slam/ros_logs' \
    -czf omni_slam_source.tar.gz omni_slam
```

If you want to package a prebuilt install tree from Orin:

```bash
cd ~/robot_ws
tar -czf omni_slam_orin_install.tar.gz install src/omni_slam/maps/map.pcd
```

On the target:

```bash
tar -xzf omni_slam_orin_install.tar.gz -C ~/robot_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/robot_ws/install/setup.bash
```

Prebuilt install trees are not portable across different ROS distro, Ubuntu release, PCL ABI, or Livox driver build. Rebuild on the target if anything differs.

## 10. Cross Compilation Option

Use cross compilation only if needed.

Required inputs:

```text
x86_64 Ubuntu host
aarch64 compiler toolchain
Orin sysroot copied from the target board
ROS 2 aarch64 libraries inside the sysroot
Livox driver aarch64 install inside the sysroot
```

Install cross compiler on host:

```bash
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu qemu-user-static rsync
```

Create sysroot from Orin:

```bash
mkdir -p ~/sysroots/orin
rsync -aAX --numeric-ids orin:/lib ~/sysroots/orin/
rsync -aAX --numeric-ids orin:/usr ~/sysroots/orin/
rsync -aAX --numeric-ids orin:/opt/ros ~/sysroots/orin/opt/
rsync -aAX --numeric-ids orin:~/ws_livox/install ~/sysroots/orin/home/user/ws_livox/
```

Create `toolchain-orin-aarch64.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_SYSROOT "$ENV{ORIN_SYSROOT}")
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

Then try:

```bash
export ORIN_SYSROOT=$HOME/sysroots/orin
source /opt/ros/humble/setup.bash

colcon build --merge-install \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/toolchain-orin-aarch64.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$ORIN_SYSROOT/opt/ros/humble;$ORIN_SYSROOT/home/user/ws_livox/install"
```

Known risk: many ROS 2 CMake packages execute host tools during build. If a package tries to execute an aarch64 binary on x86, you need qemu/binfmt or a native Orin build instead.

## 11. Recommended Code Cleanups Before Production

### Package dependency declarations

`icp_relocalization/package.xml` should declare all dependencies used by CMake/code:

```xml
<depend>sensor_msgs</depend>
<depend>pcl_conversions</depend>
<depend>tf2_ros</depend>
<depend>tf2_eigen</depend>
<depend>tf2_geometry_msgs</depend>
<depend>octomap_ros</depend>
<depend>livox_ros_driver2</depend>
```

This matters for `rosdep install` on Orin.

### Avoid hard-coded paths

The launch/config path style is mostly good now. Keep map path as a launch argument:

```bash
map_path:=/absolute/path/to/map.pcd
```

Avoid committing machine-specific paths like:

```text
/home/user/...
/home/sentry_ws/...
```

inside generic launch files.

### Runtime map location

For the robot, use a stable path such as:

```bash
~/maps/map.pcd
```

Then launch:

```bash
ros2 launch fast_lio relocalizing.launch.py map_path:=$HOME/maps/map.pcd
```

### ICP thresholds

The current test converged with fitness around `0.04` to `0.19`.

Current relocalization launch uses:

```yaml
fitness_score_thre: 0.2
converged_count_thre: 40
```

On the real dog, tune these with live data. If relocalization is slow, reduce `converged_count_thre`; if it false-positives, lower `fitness_score_thre`.

### RViz on robot

Do not run RViz on Orin during normal operation. Run RViz on a laptop and set ROS networking so it subscribes remotely.

## 12. Quick Board Run Checklist

On Orin:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source ~/robot_ws/install/setup.bash

ros2 topic list
ros2 topic echo /front_lidar --once --field header
ros2 topic echo /front_lidar/imu --once --field header
```

Mapping:

```bash
ros2 launch fast_lio mapping.launch.py
```

Relocalization:

```bash
ros2 launch fast_lio relocalizing.launch.py map_path:=$HOME/maps/map.pcd
```

Health checks:

```bash
ros2 topic hz /state_estimation
ros2 topic echo /state_estimation --once
ros2 run tf2_ros tf2_echo map odom
```

## 13. Repository Scripts Added For Deployment

This repository now includes helper scripts:

```text
scripts/sync_orin_sysroot.sh        # copy /lib, /usr, /opt/ros and Livox install from Orin into a local sysroot
scripts/build_orin_cross.sh         # cross-compile with toolchains/orin-aarch64.cmake
scripts/package_orin_runtime.sh     # package a cross-compiled install tree into a runtime tarball
scripts/build_native_orin.sh        # build natively on the Orin board
scripts/package_orin_source.sh      # create a source+map package for the Orin board
```

Current local status: `/home/user/sysroots/orin` exists, but it is not a complete target sysroot yet. It is missing:

```text
/opt/ros/humble
/usr/include/pcl-1.12
Livox driver install
```

So `scripts/build_orin_cross.sh` is configured, but it cannot produce a runnable aarch64 binary package until the target sysroot is synchronized from the Orin board.

To synchronize the sysroot from the board:

```bash
ORIN_SYSROOT=$HOME/sysroots/orin \
LIVOX_WS_ON_ORIN=$HOME/ws_livox/install \
./scripts/sync_orin_sysroot.sh user@orin-host
```

Then cross-compile with the host default `aarch64-linux-gnu-g++`:

```bash
ORIN_SYSROOT=$HOME/sysroots/orin ./scripts/build_orin_cross.sh
```

Or cross-compile with the downloaded Buildroot SDK:

```bash
ORIN_SYSROOT=$HOME/sysroots/orin \
ORIN_TOOLCHAIN_ROOT=$PWD/toolchains/aarch64--glibc--stable-final \
./scripts/build_orin_cross.sh
```

The downloaded Buildroot SDK provides GCC 9.3.0 and a minimal `aarch64` sysroot, but it does not contain the target ROS 2, PCL, or Livox installations. Keep using the Orin-synchronized sysroot as `ORIN_SYSROOT`; use `ORIN_TOOLCHAIN_ROOT` only to choose this compiler.

Then package the aarch64 runtime install tree:

```bash
MAP_PATH=$PWD/maps/map.pcd ./scripts/package_orin_runtime.sh
```

Until that sysroot exists, use the source package path:

```bash
MAP_PATH=$PWD/maps/map.pcd ./scripts/package_orin_source.sh
```

Copy the generated `dist/omni_slam_orin_source_*.tar.gz` to Orin, extract it, and run:

```bash
./scripts/build_native_orin.sh
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch fast_lio relocalizing.launch.py map_path:=$PWD/maps/map.pcd
```
