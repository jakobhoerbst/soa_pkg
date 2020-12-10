#!/bin/bash
echo ""
echo "PROJECT 2 - starting"
echo ""
echo "note 1: killing running gazebo processes"
echo "note 2: killing all running roscore & rosmaster"

killall -9 gzserver
killall -9 gzclient
killall -9 roscore
killall -9 rosmaster
rosnode kill --all
sleep 1

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
#roslaunch soa_pkg project.launch x_pos:=$x_pos y_pos:=$y_pos&
roslaunch soa_pkg project.launch x_pos:=0.625 y_pos:=0.625&
sleep 15
roslaunch soa_pkg rvizLaunch.launch&

# Start Nodes in one Terminal (--tab)
gnome-terminal --tab -e 'sh -c "rosrun soa_pkg navigation; exec bash"'
sleep 1
gnome-terminal --tab -e 'sh -c "rosrun soa_pkg main; exec bash"'


# killing all running rosnodes and gazebo client/server after pressing q
#sleep 10
#userinput=""
#echo ""
#echo "Press \"q\" key to quit and kill all"

#while read -r -n1 key
#do

   # if [[ $key == q ]] ; then
    #    break;
    #fi

#done
#printf "\nsee you\n"
#killall -9 gzserver
#killall -9 gzclient
#rosnode kill --all

#ps -ef | grep ros
#rosnode kill /gazebo

