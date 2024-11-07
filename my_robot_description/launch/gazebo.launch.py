from ament_index_python import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path
import os


def generate_launch_description():
    
    # urdf_path = os.path.join(get_package_share_path('my_robot_description'),
    #                          'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'rviz', 'urdf_config.rviz')

    robot_description_dir = get_package_share_directory("my_robot_description")
    
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        robot_description_dir, "urdf", "my_robot.urdf.xacro"),
        description="absolute path to robot urdf file")

    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(robot_description_dir).parent.resolve())]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )


    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
            launch_arguments=[
                ("gz_args", [" -v 4", " -r", " empty.sdf"])
            ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "my_robot"]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_resource_path,
        rviz2_node,
        gazebo,
        gz_spawn_entity
    ])