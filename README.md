# soa_pkg
MMR3 SOAR Project 2 - Maze Solving Algorithm with Turtlebot Waffle


## Description
Student semester project in service and objectoriented algorithms in robotics (soar) @ FHTW
See rosdoc_lite documentation in /soa_pkg/doc/html/index.html

## System Recommendations
This code is tested with two different systems: 
1. System: 
-Ubuntu 21.04 LTS 
-ROS Noetic 
-8192 MB RAM
2. System: 
-OS 
-ROS Version 
-RAM 

## Install

1. Go to your catkin workspace and download the repository  
`$ cd /PathTo/catkin_ws/src`  
`$ git clone https://github.com/jakobhoerbst/soa_pkg/`

2. Install Package via catkin_make  
`$ cd catkin_ws` or `$ cd ..`  
`$ catkin_make`

## Starting
With `testingSequence.sh` (instead of `start.sh`) the program will be executed 100 times (`start.sh` = 1 Run). 
In line 32-33 it must be selected if odometry or ground truth is used for position gathering. 
Ground Truth is pre-selected.

`$ cd /PathTo/catkin_ws/src/soa_pkg`  
`$ bash SOA.sh`

### Contribution
Code-Author: Hoerbst Jakob
Code-Contributor: Gmeiner Marc