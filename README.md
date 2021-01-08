# soa_pkg
SOAR: Turtlebot Waffle Labyrinth Algorithm


## Description
Student semester project in service and objectoriented algorithms in robotics (soar) @ FHTW
See Doxygen documentation under /docs/html/index.html

## Install

1. Go to your catkin workspace and download the rep  
`$ cd catkin_ws/src`  
`$ git clone https://github.com/jakobhoerbst/soa_pkg/`

2. Install Package via catkin_make  
`$ cd catkin_ws` or `$ cd ..`  
`$ catkin_make`

## Starting
`$ cd /PathTo/soa_pkg`  
`$ bash SOA.sh`

# How it works
See paper in /doc


## ros commands:


`rqt`       - startet rqt explorer, damit kann man topics anzeigen, schreiben und zb. auf cmd vel schreiben. 

`gzclient`  - öffnet gazebo (wenn die gui zb. im launchfile deaktiviert ist) 

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` - öffnet Teleoperator Manuelle Steuerung Turtlebot


## git commands: 

`git pull` - pull request fuer den aktuellen Stand vom git (am besten immer machen bevor man zu arbeiten beginnt. 

`git add .` - fuegt alle files hinzu (statt dem punkt können auch einzelne files ausgewaehlt werden) 

`git status` - zeigt Aenderungen vom aktuellen Stand auf deiner Maschine im Vgl. zum Git an. 

`git commit -m "hierstehtdeinKommentar"` - den folgenden push kommentieren

`git push` - push des aktuellen Stands

`git push origin [name_of_your_new_branch]` - push von dem aktuellen branch

`git checkout -b [name_of_your_new_branch]` - neuer branch auf local machine erstellen

`git checkout [branch name]` - change branch

`git branch -d localBranchName` - delete branch locally

`git push origin --delete remoteBranchName` - delete branch remotely
