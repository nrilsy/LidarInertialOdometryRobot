from launch import LaunchDescription
from ament_index_python import get_package_share_path, get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os



def generate_launch_description():

    # Define the path to your configuration file



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        # This is just a change
    )

    

    robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "my_robot_controller",
            "--controller-manager",
            "/controller_manager",
        ],  # Controller name from YAML config
    )



    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        joint_state_broadcaster_spawner,
        robot_controller
    ])