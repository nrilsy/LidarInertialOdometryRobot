# RobotTaskAssignmentSystem

This repository contains the code and resources for a 2D Lidar-Inertial Odometry Robot project. The goal of this project is to implement an odometry system that fuses data from a LiDAR sensor and an inertial measurement unit (IMU) for precise localization and navigation in Gazebo Simulation Tool. 

Prerequisites
ROS 2 Humble
Gazebo fortress
Simple-2D-LiDAR-Odometry repository

my_robot_controller package has config files for both joy_node and controller manager. DiffDriveController and JointStateBroadcaster is used in this package. 
controller manager and joint broadcaster can be launched by: ros2 launch my_robot_controller controller.launch.py
joystick controller can be launched by: ros2 launch my_robot_controller joystick_teleop.launch.py

my_robot_description package is where the urdf files and worlds are described. Also, Gazebo plugins and Lidar and IMU sensor plugins are defined (in urdf folder), and for ros2_control, necessary plugins and joint interfaces are defined.  
Gazebo simulation can be launch by: ros2 launch my_robot_description gazebo.launch.py

my_robot_firmware package is not completed yet. The goal is to write a hardware component to run DC Motors (without encoders) via Raspberry Pi GPIO pins. However, this package will probably be changed.

my_robot_localization package has the config files for lidar odometry package and robot_localization package. 
Can be launched by: ros2 launch my_robot_localization local_localization.launch.py


For Lidar odometry, the used repository is: https://github.com/dawan0111/Simple-2D-LiDAR-Odometry.git


