#!/bin/bash
echo ""
echo "PROJECT 2 - starting"
echo ""
echo "note: killing running gazebo processes"

killall -9 gzserver
killall -9 gzclient
##sleep 5

#mate-terminal -e 'sh -c "roslaunch hoerbst hoerbst.launch; exec bash"'

#roslaunch turtlebot3_bringup turtlebot3_model.launch 
export TURTLEBOT3 MODELL=waffle_pi

#sleep 5

x_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")
y_pos=$(bc <<< "scale=2;$(($RANDOM % 10))*1.25-5.625")

echo random x: $x_pos
echo random y: $y_pos

roslaunch soa_pkg project.launch x_pos:=$x_pos y_pos:=$y_pos&
sleep 15
roslaunch soa_pkg rvizLaunch.launch&

#mate-terminal -e 'sh -c "roslaunch soa_pkg rvizLaunch.launch; exec bash"'

#rviz -d rviz/rvizConfig.rviz &

#mate-terminal -e 'sh -c "rviz -d rviz/rvizConfig.rviz"'
#sleep 10
#xterm -e 'rosrun hoerbst hoerbst __name:=JACUP /JACUP/odom:=/odom /JACUP/scan:=/scan' &
#xterm -e 'roslaunch hoerbst remote.launch'

#mate-terminal -e 'sh -c "rosrun soa_pkg hoerbst; exec bash"'
#mate-terminal -e 'sh -c "rosrun soa_pkg navigation; exec bash"'
