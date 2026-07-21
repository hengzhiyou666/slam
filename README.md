# 2_slam：FAST-LIO 建图与定位

这个工程只做两件事：

- 建图：使用雷达和 IMU 建立 `maps/map.pcd`。
- 定位：加载 `maps/map.pcd`，实时输出雷达在地图中的位置。

## 启动关系

建图：

```text
1_mapping.sh
  -> FAST_LIO/launch/mapping.launch.py
  -> FAST_LIO/config/mapping.yaml
```

定位：

```text
2_relocalizing.sh
  -> FAST_LIO/launch/relocalizing.launch.py
  -> FAST_LIO/config/relocalizing.yaml
```

简单理解：

- `.sh`：准备运行环境。
- `launch.py`：启动 ROS 2 节点。
- `.yaml`：设置话题、坐标系和算法参数。

## 一、建图

输入：

```text
/lidar_points  sensor_msgs/msg/PointCloud2  雷达点云
/lidar_imu     sensor_msgs/msg/Imu          雷达 IMU
```

主要输出：

```text
/mapping/map_frame/odometry  nav_msgs/msg/Odometry  雷达实时位置
/mapping/map_frame/path      nav_msgs/msg/Path      建图轨迹
maps/map.pcd                                        点云地图文件
```

建图时的 TF 树：

```text
map_frame -> lidar_frame
```

运行：

```bash
cd /你的路径/2_slam
bash 1_mapping.sh
```

建图完成后按 `Ctrl+C`。程序正常退出时，地图保存到：

```text
2_slam/maps/map.pcd
```

## 二、定位

定位读取建图生成的 `maps/map.pcd`。

输入：

```text
/lidar_points  sensor_msgs/msg/PointCloud2  实时雷达点云
/lidar_imu     sensor_msgs/msg/Imu          实时雷达 IMU
maps/map.pcd                              先验地图
```

主要输出：

```text
/relocalizing/odom_frame/odometry  nav_msgs/msg/Odometry  连续里程计
/relocalizing/map_frame/odometry   nav_msgs/msg/Odometry  地图中的实时位置
```

定位时的 TF 树：

```text
map_frame -> odom_frame -> lidar_frame
```

运行：

```bash
cd /你的路径/2_slam
bash 2_relocalizing.sh
```

如果要指定另一张地图和大概初始位置：

```bash
MAP_PATH=/绝对路径/map.pcd bash 2_relocalizing.sh \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

## 三、适配其他机器狗

主要修改下面两个文件：

```text
FAST_LIO/config/mapping.yaml
FAST_LIO/config/relocalizing.yaml
```

两份文件需要同时检查：

```yaml
common:
    lid_topic: "/机器狗的雷达话题"
    imu_topic: "/机器狗的IMU话题"

preprocess:
    lidar_type: 雷达类型
    scan_line: 雷达线数
    scan_rate: 雷达频率

mapping:
    extrinsic_T: [雷达到IMU的平移外参]
    extrinsic_R: [雷达到IMU的旋转外参]
```

本工程自己的坐标系名称保持为：

```text
map_frame
odom_frame
lidar_frame
```

如果新雷达的 `PointCloud2` 字段格式与 dog3 的 Vanjee 雷达不同，还需要修改：

```text
FAST_LIO/src/preprocess.cpp
FAST_LIO/src/preprocess.h
```
