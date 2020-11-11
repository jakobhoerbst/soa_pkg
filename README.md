# soa_pkg
SOAR: Turtlebot Waffle Labyrinth Algorithm


# Description
short text

# Install

1. Go to your catkin workspace and download the rep
`$ cd catkin_ws/src`
`$ git clone https://github.com/jakobhoerbst/soa_pkg/`

2. Install Package via catkin_make
`$ cd catkin_ws` or `$ cd ..`
`$ catkin_make`

# How it works
textwithpictures


# ros commands: 

`roslaunch soa_pkg project.launch` - startet project

`rqt`       - startet rqt explorer, damit kann man topics anzeigen, schreiben und zb. auf cmd vel schreiben. 

`gzclient`  - öffnet gazebo (wenn die gui zb. im launchfile deaktiviert ist) 

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` - öffnet Teleoperator Manuelle Steuerung Turtlebot

# git commands: 

`git pull` - pull request fuer den aktuellen Stand vom git (am besten immer machen bevor man zu arbeiten beginnt. 

`git add .` - fuegt alle files hinzu (statt dem punkt können auch einzelne files ausgewaehlt werden) 

`git status` - zeigt Aenderungen vom aktuellen Stand auf deiner Maschine im Vgl. zum Git an. 

`git commit -m "hierstehtdeinKommentar"` - den folgenden push kommentieren

`git push` - push des aktuellen Stands

