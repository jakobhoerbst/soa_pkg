#!/bin/bash
echo ""
echo "PROJECT 2 - starting"
echo ""


for i in {1..100}
do
    echo "note 1: killing running gazebo processes"
    echo "note 2: killing all running roscore & rosmaster"
    killall -9 gzserver
    killall -9 gzclient
    killall -9 roscore
    killall -9 rosmaster
    killall -9 rviz
    rosnode kill --all
    sleep 5

    # select Robot
    export TURTLEBOT3 MODELL=waffle_pi

    # generate Random Start Position
    echo ""
    x_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")
    y_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")
    echo random x: $x_pos
    echo random y: $y_pos
    echo "" 

    # Start launchfile and load generated Start Position
    roslaunch soa_pkg project.launch x_pos:=$x_pos y_pos:=$y_pos&
    sleep 10
    roslaunch soa_pkg rvizLaunch.launch&

    sleep 5
    rosrun soa_pkg main GT&

    echo "TEST RUN: " $i 
    sleep 900
done
