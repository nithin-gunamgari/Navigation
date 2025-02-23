# SLAM on Turtlebot from scratch
* Author: Nithin Gunamgari

# Features
* Differential Drive Kinematics
* Dead Reckoning Odometry
* Implementation of EKF (Extended Kalman Filter) SLAM from scratch

# Package List
This repository consists of several ROS packages
- nurtlesim - contains simulated world in rviz for Turtlebot3
- nuslam - contains a library for extended kalman filters and provides and implementation of SLAM
- nuturtle_description - contains urdf files, basic debugging, testing, and visualization code for the Turtlebot3
- nuturtle_robot - provides an interface for hardware on the Turtlebot3
- rigid2d - provides a C++ library for 2D transformations
- nuturtle_msgs - message definition for turtlebot sensors and wheel commands (written by Matthew Elwin)<br/>
