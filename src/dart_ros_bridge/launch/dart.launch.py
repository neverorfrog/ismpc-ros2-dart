from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch argument for robot description package
        DeclareLaunchArgument(
            'robot_description_package',
            default_value='hrp4_description',
            description='Name of the robot description package'
        ),
        DeclareLaunchArgument(
            'robot_urdf_filename',
            default_value='hrp4.urdf',
            description='Name of the URDF file (without path)'
        ),
        DeclareLaunchArgument(
            'ground_urdf_filename',
            default_value='ground.urdf',
            description='Full path to the URDF file'
        ),
        
        # Launch the dart_ros_bridge node
        Node(
            package='dart_ros_bridge',
            executable='dart_ros_bridge_exec',
            name='dart_ros_bridge',
            parameters=[{
                'robot_description_package': LaunchConfiguration('robot_description_package'),
                'robot_urdf_filename': LaunchConfiguration('robot_urdf_filename'),
                'ground_urdf_filename': LaunchConfiguration('ground_urdf_filename')
            }]
        )
    ])