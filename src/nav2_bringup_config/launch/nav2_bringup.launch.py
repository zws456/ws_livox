import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config_dir = get_package_share_directory('nav2_bringup_config')
    
    params_file = os.path.join(nav2_config_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    
    # Nav2 navigation launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'map': map_yaml_file,
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed bringup if True'),
        nav2_launch,
    ])
