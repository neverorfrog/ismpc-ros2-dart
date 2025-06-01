import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    description_path = FindPackageShare(package='hrp4_description').find('hrp4_description')
    robot_model_path = os.path.join(description_path, 'urdf', 'hrp4.urdf')
    rviz_config_path = os.path.join(get_package_share_directory("dart_ros_bridge"),"config","config.rviz")
    
    dart_ros_bridge_node = Node(
        package='dart_ros_bridge',
        executable='dart_ros_bridge_exec',
        name='dart_ros_bridge',
        parameters=[{
            'robot_description_package': LaunchConfiguration('robot_description_package'),
            'robot_urdf_filename': LaunchConfiguration('robot_urdf_filename'),
            'ground_urdf_filename': LaunchConfiguration('ground_urdf_filename')
        }]
    )
    
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )
    
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    
    # robot_state_publisher_node = TimerAction(
    #     period=3.0,
    #     actions=[
    #         Node(
    #             package='robot_state_publisher',
    #             executable='robot_state_publisher',
    #             parameters=[{
    #                 'robot_description': ParameterValue(
    #                     Command(['xacro ', robot_model_path]), 
    #                     value_type=str
    #                 )
    #             }]            
    #         )
    #     ]
    # )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
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
        DeclareLaunchArgument(
            'rvizconfig', 
            default_value=rviz_config_path, 
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            'model', 
            default_value=robot_model_path, 
            description='Absolute path to robot model file'
        ),
        DeclareLaunchArgument(
            'gui', 
            default_value='True', 
            description='Flag to enable joint_state_publisher_gui'
        ),
        
        dart_ros_bridge_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        # robot_state_publisher_node,
        rviz_node

    ])