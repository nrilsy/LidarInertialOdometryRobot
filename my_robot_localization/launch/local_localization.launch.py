from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    lidar_odometry_node = Node(
        package="lidar_odometry",
        executable="lidar_odometry_node",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "lidar.yaml")],
        output="screen",
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("my_robot_localization"), "config", "ekf.yaml")]
    )

    return LaunchDescription([
        lidar_odometry_node,
        robot_localization,
    ])