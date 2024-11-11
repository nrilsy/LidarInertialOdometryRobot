from launch import LaunchDescription
from ament_index_python import get_package_share_path, get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os



def generate_launch_description():

    # Define the path to your configuration file
    config_file_path = os.path.join(
        get_package_share_directory('my_robot_controller'),
        'config',
        'my_robot_controllers.yaml'
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    

    robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "my_robot_controller",
            "--controller-manager",
            "/controller_manager"
        ],  # Controller name from YAML config
        output='screen',
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