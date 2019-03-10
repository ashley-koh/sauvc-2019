# NPES SAUVC
Git Repository for NPES SAUVC!

Use this repository to keep track and update the AUV Project for the annual [SAUVC](https://sauvc.org/) event.

If you are new to all of this, please click here for links to learn.

## Requirements
Operating System: [Ubuntu 16.04 (Xenial Xerus) Desktop](http://releases.ubuntu.com/16.04/).

Installation for Dual-Boot by either [partitioning your drive](https://www.tecmint.com/install-ubuntu-16-04-alongside-with-windows-10-or-8-in-dual-boot/) or [making a bootable USB drive](https://www.youtube.com/watch?v=YIhYitXwJfE).

### Software
1. [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) or newer
    * [bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground)
    * [cv_camera](http://wiki.ros.org/cv_camera)
    * [mavros](http://wiki.ros.org/mavros)
2. Python
    * [Pip](https://pypi.org/project/pip/)
    * [OpenCV](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
    * [NumPy](http://www.numpy.org/)
3. Gazebo* (for simulation only)
    * [freefloating_gazebo](https://github.com/freefloating-gazebo/freefloating_gazebo)
4. Git
    
Asterisk * (Optional)

## Installation
1. Update package list and system software to lastest version:
   * `sudo apt-get update && sudo apt-get -y upgrade`
1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation):
   * Select Ubuntu as your platform
   * Install ros-kinetic-full-desktop
2. Install mavros:
   * `sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras`
   * run `install_geographiclib_datasets.sh`
   * If you cannot find it, get it using `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh`
3. Install pip:
   * `sudo apt-get install python-pip`
   * Check if installed by running `pip -V'
4. Install OpenCV and NumPy:
   * `pip install opencv-python numpy`
5. Install Git
   * `apt-get install git-core`
   * Chech if installed by running `git -V`
    
### Cloning (Only after ROS is installed)
1. Open Terminal by pressing Ctrl + Alt + T
2. Git Clone this repository to an arbitrary directory:
   * `git clone https://github.com/ashley-koh/sauvc-2019`
3. Go into copied repository:
   * `cd sauvc-2019`
4. Open Files Application
5. Drag and drop following folders into your `~/catkin_ws/src` directory:
   * opencv_converter
   * controller
6. Go into ROS package source directory in terminal:
   * `cd ~/catkin_ws/src`
7. Clone bluerov_ros_playground into the directory:
   * `git clone https://github.com/patrickelectric/bluerov_ros_playground`
8. Clone cv_camera into the same directory:
   * `git clone https://github.com/OTL/cv_camera.git`
9. Build and Install
   * `catkin_make`

## Usage
1. Open Terminal using Ctrl + Alt + T
2. Go into ROS workspace directory:
   * `cd catkin_ws`
3. Source setup file
   * `source devel/setup.bash'
   * Make terminal run command whenever you open a new terminal by running `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
4. Run Master
   * `roscore`
5. 

## Contributing to NPES SAUVC
Contact Ashley:
HP: +65 92992543
Email: ashleykoh24@gmail.com

## Credits
* [Ashley Koh Jia Jhin](https://github.com/ashley-koh)
* [Zhu Zhan Yan](https://github.com/mrzzy)
* [Tan Khay Liang](https://github.com/khayliang)

## LICENSE
NPES SAUVC is under the MIT License
