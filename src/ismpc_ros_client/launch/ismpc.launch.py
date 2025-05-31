from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ismpc_ros_client',
            executable='ismpc_ros_client_exec',
            name='ismpc_ros_client'
        )
    ])
