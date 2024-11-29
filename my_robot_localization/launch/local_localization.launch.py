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

    rf2o_odometry = Node(
        package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 5.0}],
                )

    return LaunchDescription([
        # rf2o_odometry,
        lidar_odometry_node,
        robot_localization,
    ])