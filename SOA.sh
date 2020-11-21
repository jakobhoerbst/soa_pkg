#!/bin/bash
echo "PROJECT 2 - startung"
echo "note: killing running gazebo processes"

killall -9 gzserver
killall -9 gzclient
sleep 5

#mate-terminal -e 'sh -c "roslaunch hoerbst hoerbst.launch; exec bash"'
roslaunch soa_pkg project.launch &
sleep 10
rviz -d rviz/rvizConfig.rviz &

sleep 5
#xterm -e 'rosrun hoerbst hoerbst __name:=JACUP /JACUP/odom:=/odom /JACUP/scan:=/scan' &
#xterm -e 'roslaunch hoerbst remote.launch'

mate-terminal -e 'sh -c "rosrun soa_pkg hoerbst; exec bash"'
