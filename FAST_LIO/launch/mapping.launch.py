# =============================================================================
# 这个文件是做什么的？
# =============================================================================
#
# 这是一个 ROS 2 的 launch（启动）文件，作用是启动 FAST-LIO 建图节点。
#
# 可以把它想象成一张“开机任务清单”：
#
#   1. 找到 fast_lio 软件包放在哪里；
#   2. 找到建图参数文件 mapping.yaml；
#   3. 准备好可以从命令行修改的参数；
#   4. 启动真正负责建图的 fastlio_mapping 程序。
#
# 当前工程的完整启动关系是：
#
#   ./1_mapping.sh
#          ↓
#   mapping.launch.py（就是当前文件）
#          ↓
#   mapping.yaml（雷达话题、IMU 话题、外参、滤波参数等）
#          ↓
#   fastlio_mapping（真正执行 FAST-LIO 算法的 C++ 程序）
#
# 平时可以在 2_slam 目录中运行：
#
#   ./1_mapping.sh
#
# 它最终相当于执行：
#
#   ros2 launch fast_lio mapping.launch.py
#
# 注意：这个文件只启动 FAST-LIO 建图节点。
# 它不会帮我们启动雷达驱动、IMU 驱动或 RViz。
# 所以在开始建图前，雷达和 IMU 应该已经在发布数据。
# =============================================================================


# 导入 Python 自带的路径处理工具。
# 我们会用它把“软件包目录”“config 目录”和“mapping.yaml 文件名”
# 安全地拼成一个完整路径。
import os.path

# ROS 2 软件包经过 colcon build 以后，会被安装到 install 目录。
# get_package_share_directory() 可以向 ROS 2 询问：
# “fast_lio 软件包安装后的 share 目录在哪里？”
# 这样就不用在代码中写死 /home/xxx/... 之类的绝对路径。
from ament_index_python.packages import get_package_share_directory

# LaunchDescription 可以理解成一个“启动任务盒子”。
# 最后要把所有需要执行的任务放进这个盒子，再交给 ROS 2。
from launch import LaunchDescription

# DeclareLaunchArgument 用来声明一个可以从命令行传入的参数。
# 例如：use_sim_time:=true 或 config_file:=/某个路径/参数.yaml。
from launch.actions import DeclareLaunchArgument

# LaunchConfiguration 表示“这个值等到真正启动时再确定”。
# 它会读取用户从命令行传入的值；如果用户没有传，就使用默认值。
from launch.substitutions import LaunchConfiguration

# Node 表示要启动一个 ROS 2 节点（也就是一个正在运行的 ROS 2 程序）。
from launch_ros.actions import Node


# ROS 2 执行 launch 文件时，会主动寻找并调用这个函数。
# 函数名 generate_launch_description 是 ROS 2 规定好的，不能随便改名。
# 这个函数最后必须返回一个 LaunchDescription。
def generate_launch_description():
    # -------------------------------------------------------------------------
    # 第一步：找到默认的建图 YAML 参数文件
    # -------------------------------------------------------------------------

    # 获取 fast_lio 软件包安装后的 share 目录。
    # 例如，它可能是：
    #   /home/用户名/工作空间/install/fast_lio/share/fast_lio
    #
    # 不同电脑上的实际路径可能不同，所以这里让 ROS 2 自动寻找。
    package_path = get_package_share_directory('fast_lio')

    # 在 fast_lio 软件包目录后面依次加上：
    #   config/mapping.yaml
    #
    # 最终得到默认建图参数文件的完整路径，例如：
    #   .../install/fast_lio/share/fast_lio/config/mapping.yaml
    #
    # mapping.yaml 中包含：
    #   - 输入点云话题 /lidar_points；
    #   - 输入 IMU 话题 /lidar_imu；
    #   - 雷达与 IMU 的外参；
    #   - 点云滤波参数；
    #   - 地图发布和 PCD 保存开关等。
    default_config_file = os.path.join(
        package_path, 'config', 'mapping.yaml')

    # -------------------------------------------------------------------------
    # 第二步：准备两个可以从命令行修改的启动参数
    # -------------------------------------------------------------------------

    # use_sim_time 是“是否使用仿真时间”。
    #
    # false：使用正常的 ROS/系统时间，真机运行一般使用这个值。
    # true ：使用 /clock 话题提供的时间，Gazebo 等仿真环境常用这个值。
    #
    # LaunchConfiguration 此时只是先放一个“参数占位符”；
    # 真正的值会在启动时从下面的 DeclareLaunchArgument 中取得。
    use_sim_time = LaunchConfiguration('use_sim_time')

    # config_file 表示 FAST-LIO 要读取哪个 YAML 参数文件。
    # 默认使用上面找到的 mapping.yaml，但也允许启动时换成其他文件。
    config_file = LaunchConfiguration('config_file')

    # 正式声明 use_sim_time 启动参数。
    # 如果用户没有在命令行指定它，就使用字符串 'false'。
    #
    # 真机建图时通常直接运行：
    #   ./1_mapping.sh
    #
    # 仿真建图时可以运行：
    #   ./1_mapping.sh use_sim_time:=true
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    # 正式声明 config_file 启动参数。
    # 如果用户没有指定其他文件，就使用 config/mapping.yaml。
    #
    # 如果临时想测试另一份 YAML，可以这样运行：
    #   ./1_mapping.sh config_file:=/绝对路径/其他参数.yaml
    #
    # 这样只是本次启动使用另一份文件，不会修改这里的默认设置。
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Yaml config file path')

    # -------------------------------------------------------------------------
    # 第三步：告诉 ROS 2 要启动哪个程序
    # -------------------------------------------------------------------------

    # 创建 FAST-LIO 节点的启动任务。
    # 注意：这里还没有立刻启动程序，只是在准备一张“节点启动卡片”。
    fast_lio_node = Node(
        # package 表示程序属于哪个 ROS 2 软件包。
        package='fast_lio',

        # executable 表示真正运行的可执行程序名称。
        # fastlio_mapping 由 FAST_LIO/src/laserMapping.cpp 等 C++ 源码编译得到，
        # 点云与 IMU 的融合、里程计计算和地图构建主要都在这个程序中完成。
        executable='fastlio_mapping',

        # parameters 是传给 fastlio_mapping 节点的参数列表。
        parameters=[
            # 首先读取整份 mapping.yaml。
            config_file,

            # 再单独传入 use_sim_time。
            # 如果 YAML 中也存在同名参数，这里的值会作为启动时的设置使用。
            {'use_sim_time': use_sim_time},
        ],

        # 把节点运行时的日志输出到当前终端。
        # 这样可以看到启动成功、参数错误、收不到点云等提示信息。
        output='screen',

        # 话题重映射可以理解成给话题“改名字”。
        # FAST-LIO 源码原本向 /Odometry 发布里程计，
        # 这里把外部实际看到的话题改成 /mapping/odom_frame/odometry。
        #
        # 左边：程序源码中使用的原话题名。
        # 右边：运行这个 launch 文件后实际使用的话题名。
        #
        # 这个设置只是在修改输出里程计话题，
        # 不会修改 /lidar_points 和 /lidar_imu 输入话题；
        # 两个输入话题是在 mapping.yaml 中设置的。
        remappings=[('/Odometry', '/mapping/odom_frame/odometry')])

    # -------------------------------------------------------------------------
    # 第四步：把前面准备好的任务放进启动任务盒子
    # -------------------------------------------------------------------------

    # 创建一个空的启动任务清单。
    ld = LaunchDescription()

    # 把 use_sim_time 参数声明放进清单。
    # ROS 2 看到它以后，才知道启动命令可以接收 use_sim_time:=...。
    ld.add_action(declare_use_sim_time_cmd)

    # 把 config_file 参数声明放进清单。
    # ROS 2 看到它以后，才知道启动命令可以接收 config_file:=...。
    ld.add_action(declare_config_file_cmd)

    # 把 FAST-LIO 节点放进清单。
    # 执行到这个任务时，ROS 2 才会真正启动 fastlio_mapping 程序。
    ld.add_action(fast_lio_node)

    # 把完整的启动任务清单交还给 ROS 2。
    return ld
