#!/bin/bash
echo ""
echo "PROJECT 2 - starting"
echo ""
echo "note: killing running gazebo processes"

killall -9 gzserver
killall -9 gzclient
##sleep 5

export TURTLEBOT3 MODELL=waffle_pi

x_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")
y_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")

echo random x: $x_pos
echo random y: $y_pos

#roslaunch soa_pkg project.launch x_pos:=$x_pos y_pos:=$y_pos&
roslaunch soa_pkg project.launch x_pos:=0.625 y_pos:=0.625&
sleep 15
roslaunch soa_pkg rvizLaunch.launch&


gnome-terminal -e 'sh -c "rosrun soa_pkg hoerbst; exec bash"'
gnome-terminal -e 'sh -c "rosrun soa_pkg navigation; exec bash"'
