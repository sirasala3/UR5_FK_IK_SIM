# ~/sixDOF_robot_ws/src/ur5_motion/launch/ur5_full_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Include the MoveIt launch file
    # This will start RViz, MoveIt, and the fake robot driver
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur5',
            'use_fake_hardware': 'true'
        }.items()
    )

    # 2. Add your main.py controller node
    controller_node = Node(
        package='ur5_motion',         # Your package name
        executable='main.py',       # The name of your Python script
        name='ur5_interactive_controller',
        output='screen',
        emulate_tty=True, # Ensures Python's print statements appear
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        moveit_launch,
        controller_node
    ])
