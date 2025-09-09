from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5_motion',
            executable='ur5_controller',  # console_script in setup.py
            name='ur5_controller',
            output='screen'
        )
    ])
