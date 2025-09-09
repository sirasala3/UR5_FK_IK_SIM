from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        Node(
            package='ur_description',
            executable='view_ur.launch.py',
            name='ur5_view',
            output='screen',
            parameters=[{'ur_type': ur_type}]
        )
    ])
