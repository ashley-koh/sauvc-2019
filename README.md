# sauvc-2019
Git repo for SAUVC-2019

##Installation
1. Download [Desktop-Full install of ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. Create catkin workspace [catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
3. Clone this branch into your ~/catkin_ws/src folder using `git clone -b ros-src-folder https://github.com/ashley-koh/sauvc-2019.git`.
4. `cd ~/catkin_ws` and then run `catkin_make`.
5. Make sure to run `source ~/catkin_ws/devel/setup.bash` before running [ROS commands](http://wiki.ros.org/ROS/CommandLineTools) below.

NOTE: If you don't have [bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground) or [cv_camera](https://github.com/OTL/cv_camera), it is because they are existing ROS packages. Use the provided links to download them.

##Usage

Run the `StartROSOpenCV.sh` Shell Script from your terminal.

You can also do the following:

1. Open new terminal using Ctrl + Alt + T.
2. Run `roscore` on that new terminal.
3. Run `rosrun cv_camera cv_camera_node` on a new terminal.
4. Run `rosrun image-converter image-conversion.py` on a new terminal as well.
5. Run `roslaunch bluerov_ros_playground bluerov2_node.launch"

##Notes

`StartROSOpenCV.sh` Shell Script is running on startup on the AUV.
