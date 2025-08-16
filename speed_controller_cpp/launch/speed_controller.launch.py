#!/usr/bin/env python3
# launch/speed_controller.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    speed_profile_rate_arg = DeclareLaunchArgument(
        'speed_profile_rate',
        default_value='0.5',
        description='Speed profile change rate (m/s per second)'
    )
    
    profile_update_frequency_arg = DeclareLaunchArgument(
        'profile_update_frequency', 
        default_value='10.0',
        description='Speed profile update frequency (Hz)'
    )
    
    enable_dual_camera_arg = DeclareLaunchArgument(
        'enable_dual_camera',
        default_value='false',
        description='Enable dual camera detection fusion (FL + FR)'
    )

    # Speed controller node
    speed_controller_node = Node(
        package='speed_controller_cpp',
        executable='speed_controller_node',
        name='speed_profile_controller',
        output='screen',
        parameters=[{
            'speed_profile_rate': LaunchConfiguration('speed_profile_rate'),
            'profile_update_frequency': LaunchConfiguration('profile_update_frequency'),
            'enable_dual_camera': LaunchConfiguration('enable_dual_camera'),
        }],
        remappings=[
            # Remap Autoware's trajectory topic so we can intercept it
            ('/planning/scenario_planning/trajectory_original', '/planning/scenario_planning/trajectory'),
            # USB Camera and TensorRT YOLOX integration
            ('/camera/image_raw', '/image_raw'),
            ('/perception/objects', '/perception/object_recognition/detection/rois0'),
        ]
    )

    return LaunchDescription([
        speed_profile_rate_arg,
        profile_update_frequency_arg,
        enable_dual_camera_arg,
        speed_controller_node,
    ])