#!/bin/bash

# NOTE:
#  -Make sure you add "source ~/catkin_ws/devel/setup.bash" into your ~/.bashrc file
#   by running [ echo "source ~/catkin_ws/devel/setup.bash">> ~/.bashrc ]

# cd into ~/catkin_ws
cd $HOME

# Start roscore in new terminal
gnome-terminal --window-with-profile=ROS_STARTUP -e "roscore" &
sleep 1s

# Run cv_camera in new terminal
gnome-terminal --window-with-profile=ROS_STARTUP -e "rosrun cv_camera cv_camera_node"
sleep 1s

# Run image-converter in new terminal
gnome-terminal --window-with-profile=ROS_STARTUP -e "rosrun image-converter image-conversion.py"
sleep 1s

# Run bluerov2_node in new terminal
gnome-terminal --window-with-profile=ROS_STARTUP -e "roslaunch bluerov_ros_playground bluerov2_node.launch"
sleep 1s

# Check for rostopics
sleep 1s
gnome-terminal --tab -e "rostopic list" --tab -e "rosnode list"

