#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    camera_mode = LaunchConfiguration('camera_mode')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'arducam'
    )
    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value = 'depth'
    )

    # Camera node
    camera_node = Node(
        package = 'arducam_tof_cam',
        namespace = robot_ns,
        executable = 'camera',
        name = 'arducam_tof_camera_node',
        parameters=[{
            'mode': LaunchConfiguration('camera_mode')
        }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        camera_mode_arg,
        camera_node
    ])
