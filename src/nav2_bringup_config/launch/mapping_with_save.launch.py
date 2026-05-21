import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 复用现有的 realtime_mapping（FAST_LIO + OctoMap）
    realtime_mapping_launch = os.path.join(
        get_package_share_directory('fast_lio'),
        'launch',
        'realtime_mapping.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realtime_mapping_launch)
        ),
    ])
