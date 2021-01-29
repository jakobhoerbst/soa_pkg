# soa_pkg
MMR3 SOAR Project 2 - Maze Solving Algorithm with Turtlebot Waffle


## Description
Student semester project in service and object oriented algorithms in robotics (soar) @ FHTW
This package provides a code documentation to the paper "Maze Solving Algorithm" by Gmeiner Marc and Jakob Hoerbst.
This code documentation is build with rosdoc_lite and can be found in /soa_pkg/doc/html/index.html

## System Recommendations
This code is tested with two different systems: 
1. System: 
-Ubuntu 20.04 LTS 
-ROS Noetic 
-8192 MB RAM
2. System: 
-Ubuntu 18.04 LTS
-ROS Melodic 
-16 GB RAM  

## Installation
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

## Legend
Meaning of the numbers in the console: \n
 * 0 ... DEAD END \n
 * 1 ... right \n
 * 2 ... down \n
 * 3 ... left \n
 * 4 ... up

### Contribution
Code-Author: Hoerbst Jakob
Code-Contributor: Gmeiner Marc
