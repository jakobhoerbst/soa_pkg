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
-Ubuntu 18.04 LTS
-ROS Melodic 
-16 GB RAM  

## Install

1. Go to your catkin workspace and download the repository  
`$ cd /PathTo/catkin_ws/src`  
`$ git clone https://github.com/jakobhoerbst/soa_pkg/`

2. Install Package via catkin_make  
`$ cd catkin_ws` or `$ cd ..`  
`$ catkin_make`

## Starting
For a single run: 
`$ cd /PathTo/catkin_ws/src/soa_pkg`  
`$ bash start.bash`
(For changing from GT to odom comment line 32 and decomment line 33 in start.bash)

For starting series of 100 runs: 
`$ cd /PathTo/catkin_ws/src/soa_pkg`  
`$ bash 100timesStart.bash`
(For changing from GT to odom edit line 37 in 100timesStart.bash)


### Contribution
Code-Author: Hoerbst Jakob
Code-Contributor: Gmeiner Marc
