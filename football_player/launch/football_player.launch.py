from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'football_player'
    config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "params",
        "controller_gains.yaml"
    ])
    pt_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "params",
        "best.pt"
    ])
    controller_node = Node(
        package=pkg_name,
        executable="follow_and_avoid",
        name="follower_and_avoider",
        output="screen",
        parameters=[config_path]
    )
    detection_and_estimation_node = Node(
        package=pkg_name,
        executable="detector",
        name="detector_node",
        output="screen",
        parameters=[{'pt_path': pt_path}]
    )
    camera_node = Node(
        package = 'v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{'video_device': '/dev/video4'}]
    )
    return LaunchDescription([
        controller_node,
        detection_and_estimation_node,
        camera_node
    ])
