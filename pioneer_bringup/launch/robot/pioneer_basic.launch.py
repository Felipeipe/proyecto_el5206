from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
# This launch file launches the base controller and a teleop node

def generate_launch_description():
    base_pkg = FindPackageShare('pioneer_base')
    joy_pkg = FindPackageShare('pioneer_joy')

    twist_mux_params = PathJoinSubstitution([
        joy_pkg,
        'params',
        'twist_mux.yaml'
    ])
    
    rosaria2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                base_pkg,
                'launch',
                'rosaria2.launch.py'
            ])
        )
    )
   
    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                joy_pkg,
                'launch',
                'joystick.launch.py'
            ])
        )
    ) 

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/cmd_vel')]
    )
    return LaunchDescription([
        joy_node,
        rosaria2_node,
        twist_mux
    ])
