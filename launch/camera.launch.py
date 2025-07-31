#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')
    frame_rate = LaunchConfiguration('frame_rate')
    flip_image = LaunchConfiguration('flip_image')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'rp0'
    )
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value = '320'
    )
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value = '240'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value = '15'
    )
    flip_image_arg = DeclareLaunchArgument(
        'flip_image',
        default_value = 'True'
    )

    # Camera node
    camera_node = Node(
        package = 'rpi_cam_ros2',
        namespace = robot_ns,
        executable = 'camera',
        name = 'rpi_camera_sensor',
        parameters=[{
            'width': LaunchConfiguration('frame_width'),
            'height': LaunchConfiguration('frame_height'),
            'framerate': LaunchConfiguration('frame_rate'),
            'codec': 'mjpeg',
            'flip': LaunchConfiguration('flip_image'),
        }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        frame_width_arg,
        frame_height_arg,
        frame_rate_arg,
        flip_image_arg,
        camera_node
    ])
