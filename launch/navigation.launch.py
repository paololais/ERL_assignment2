import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_assignment2 = get_package_share_directory('assignment2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Nav2 Navigation (Planner + Controller)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_assignment2, 'config', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(navigation_launch)
    return ld