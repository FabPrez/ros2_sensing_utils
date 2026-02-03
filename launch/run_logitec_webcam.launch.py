#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Absolute path to your venv Python interpreter
    # python_bin = "/home/fabioprez/projects/template_ws/try_venv/bin/python3"

    return LaunchDescription([
        Node(
            package='sensing_utils',
            executable='logitec_c270_publisher.py',
            name='logitec_publisher',
            output='screen',
            # remappings=[('py_template_topic', 'altro_topic')],  # se vuoi remapping
            # prefix=python_bin
        )
    ])

