#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch args
    device_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='Camera device index (es. 0, 1, ... or Linux /dev path index mapping)'
    )
    hz_arg = DeclareLaunchArgument(
        'publish_hz',
        default_value='30.0',
        description='Publish rate in Hz'
    )

    device_cfg = LaunchConfiguration('device_index')
    hz_cfg = LaunchConfiguration('publish_hz')

    node = Node(
        package='sensing_utils',                 # cambia con il nome del tuo package
        executable='logitec_c270_publisher.py',  # o il nome dell'executable installato
        name='logitec_publisher',
        output='screen',
        parameters=[{
            'device_index': device_cfg,
            'publish_hz': hz_cfg
        }]
    )

    return LaunchDescription([
        device_arg,
        hz_arg,
        node
    ])
