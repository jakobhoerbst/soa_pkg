#!/bin/bash
echo ""
echo "PROJECT 2 - starting"
echo ""
echo "note: killing running gazebo processes"

killall -9 gzserver
killall -9 gzclient
sleep 5

#mate-terminal -e 'sh -c "roslaunch hoerbst hoerbst.launch; exec bash"'

#roslaunch turtlebot3_bringup turtlebot3_model.launch 
export TURTLEBOT3 MODELL=waffle_pi

#sleep 5
roslaunch soa_pkg project.launch &
#sleep 15
#rviz -d rviz/rvizConfig.rviz &

#mate-terminal -e 'sh -c "rviz -d rviz/rvizConfig.rviz"'
#sleep 10
#xterm -e 'rosrun hoerbst hoerbst __name:=JACUP /JACUP/odom:=/odom /JACUP/scan:=/scan' &
#xterm -e 'roslaunch hoerbst remote.launch'

mate-terminal -e 'sh -c "rosrun soa_pkg hoerbst; exec bash"'
