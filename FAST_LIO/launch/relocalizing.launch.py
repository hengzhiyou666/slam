import os

# 本文件用于启动“先验地图定位”流程，主要包含三个节点：
# 1. icp_node：将当前雷达点云与已有 PCD 地图配准，计算初始位置；
# 2. fastlio_mapping：读取 ICP 结果，在已有地图中持续定位；
# 3. transform_publisher：发布 map -> odom TF，并把 odom 位姿转换到 map。
#
# 运行入口通常是：
#   ./2_relocalizing.sh
# 该脚本会把地图路径通过 map_path 参数传进来。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 找到 fast_lio 安装后的共享目录，用于定位默认 YAML 文件。
    fast_lio_share = get_package_share_directory('fast_lio')

    # 以下变量的实际值可以在启动命令中通过“参数名:=值”进行修改。
    config_file = LaunchConfiguration('config_file')
    map_path = LaunchConfiguration('map_path')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_yaw = LaunchConfiguration('initial_yaw')

    # FAST-LIO 定位模式默认使用的参数文件。
    default_config_file = os.path.join(
        fast_lio_share, 'config', 'relocalizing.yaml')

    # 声明启动参数：配置文件、先验地图和初始位姿。
    # map_path 会同时传给 ICP 和 FAST-LIO，保证两者使用同一张地图。
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Fast-LIO relocalization parameter file')
    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Prior PCD map path used by both ICP and Fast-LIO')
    declare_initial_x = DeclareLaunchArgument('initial_x', default_value='0.0')
    declare_initial_y = DeclareLaunchArgument('initial_y', default_value='0.0')
    declare_initial_z = DeclareLaunchArgument('initial_z', default_value='0.0')
    declare_initial_yaw = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    # 发布 map_frame -> odom_frame 坐标变换，并将 FAST-LIO 的连续里程计
    # 转换到 map_frame。
    map_odom_trans = Node(
        package='icp_relocalization',
        executable='transform_publisher',
        name='transform_publisher',
        parameters=[
            {'map_frame_id': 'map_frame'},
            {'odom_frame_id': 'odom_frame'},
            {'sensor_frame_id': 'lidar_frame'},
            {'icp_result_topic': '/icp_result'},
            {'input_odometry_topic': '/relocalizing/odom_frame/odometry'},
            {'output_odometry_topic': '/relocalizing/map_frame/odometry'},
        ],
        output='screen')

    # ICP 初始定位节点：用实时点云与先验 PCD 地图进行匹配。
    icp_node = Node(
        package='icp_relocalization',
        executable='icp_node',
        name='icp_node',
        output='screen',
        parameters=[
            # 机器人在地图中的初始位置和偏航角，可通过启动参数覆盖。
            {'initial_x': ParameterValue(initial_x, value_type=float)},
            {'initial_y': ParameterValue(initial_y, value_type=float)},
            {'initial_z': ParameterValue(initial_z, value_type=float)},
            {'initial_a': ParameterValue(initial_yaw, value_type=float)},

            # ICP 点云降采样与匹配参数。
            {'map_voxel_leaf_size': 0.2},
            {'cloud_voxel_leaf_size': 0.2},
            {'map_frame_id': 'map_frame'},
            {'solver_max_iter': 100},
            {'max_correspondence_distance': 1.0},
            {'RANSAC_outlier_rejection_threshold': 0.5},
            {'map_path': map_path},

            # 连续达到匹配要求后，认为初始定位成功。
            {'fitness_score_thre': 0.05},
            {'converged_count_thre': 5},

            # 机器狗雷达发布标准 PointCloud2，话题为 /lidar_points。
            {'pcl_type': 'pointcloud2'},
            {'pointcloud_topic': '/lidar_points'},
        ])

    # FAST-LIO 定位节点：加载 relocalizing.yaml，并使用同一张先验地图。
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            config_file,
            {'prior_map_path': map_path},
        ],
        output='screen',
        # 定位模式持续输出：lidar_frame 相对于 odom_frame 的里程计。
        remappings=[
            ('/Odometry', '/relocalizing/odom_frame/odometry'),
            ('/path', '/relocalizing/odom_frame/path'),
        ])

    # 等待 5 秒再启动 ICP 和 FAST-LIO，让坐标变换等基础组件先准备好。
    delayed_start_lio = TimerAction(
        period=5.0,
        actions=[
            icp_node,
            fast_lio_node,
        ])

    # 把所有参数声明和节点加入 ROS 2 启动清单。
    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(declare_map_path)
    ld.add_action(declare_initial_x)
    ld.add_action(declare_initial_y)
    ld.add_action(declare_initial_z)
    ld.add_action(declare_initial_yaw)
    ld.add_action(map_odom_trans)
    ld.add_action(delayed_start_lio)
    return ld
