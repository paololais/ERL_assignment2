import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_assignment2 = get_package_share_directory('assignment2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 1. Map Argument
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_assignment2, 'maps', 'map_of_world.yaml'),
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. Launch Nav2 Localization (AMCL + Map Server)
    # We add the 'initial_pose' arguments here to match the spawn point!
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_assignment2, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
            # AUTOMATIC INITIALIZATION:
            'initial_pose_x': '0.0',      # Matches spawn_robot_aruco.launch.py
            'initial_pose_y': '-1.0',     # Matches spawn_robot_aruco.launch.py
            'initial_pose_z': '0.0',
            'initial_pose_yaw': '-1.5707' # Matches spawn_robot_aruco.launch.py
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(map_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(localization_launch)
    return ld