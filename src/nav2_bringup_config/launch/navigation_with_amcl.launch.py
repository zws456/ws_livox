import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config_dir = get_package_share_directory('nav2_bringup_config')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # === 参数声明 ===
    map_yaml_file = LaunchConfiguration('map', default='')
    # 判断 map 参数是否非空
    map_not_empty = PythonExpression(["'", map_yaml_file, "' != ''"])
    map_empty = PythonExpression(["'", map_yaml_file, "' == ''"])

    # 坐标系统一为 body，不再需要通过 static TF 转换 body -> base_link

    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    pc2laser_config = os.path.join(nav2_config_dir, 'config', 'pointcloud_to_laserscan.yaml')

    # 1. Livox 驱动
    livox_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',
        'msg_MID360_launch.py'
    )

    # 2. FAST_LIO（纯里程计，不启动 OctoMap）
    fast_lio_launch_path = os.path.join(
        fast_lio_dir,
        'launch',
        'mid360.launch.py'
    )

    # RViz 配置路径
    rviz_config_path = os.path.join(
        fast_lio_dir,
        'rviz',
        'fastlio.rviz'
    )

    # 3. pointcloud_to_laserscan（3D 点云 → 2D scan，输出到 base_link 水平坐标系）
    pc2laser_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[pc2laser_config],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),   # FAST_LIO 输出的 body 坐标系点云
            ('scan', '/scan'),                         # 输出 2D 激光
        ]
    )

    # 4. AMCL + map_server（全局定位）— 仅在 map 参数非空时启动
    # 由 localization_launch.py 统一启动 amcl 和 map_server，避免重复
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'map': map_yaml_file,
        }.items(),
        condition=IfCondition(map_not_empty)
    )

    # 5. Nav2 导航（纯导航，不启动 map_server，避免重复）
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'False',
            'map': '',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load. If empty, runs in realtime mode without AMCL.'),


        # 启动传感器 + 里程计
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch_path),
            launch_arguments={'use_rviz': 'false'}.items()
        ),

        # 3D → 2D
        pc2laser_node,

        # 地图 + 定位（仅静态地图模式）
        amcl_launch,

        # 导航（两种模式都需要）
        nav2_launch,

        # RViz 最后启动，确保 map_server 等所有节点就绪后再打开
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])
