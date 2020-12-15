# Gazebo Scorpio Model Plugin
This plugin is for simulating Scorpio robot in Gazebo with PnC controller.
The package was tested on ROS Kinetic Ubuntu 16.04 LTS. Gazebo 9.

## Clone the repository
Make sure [Git Lfs](https://git-lfs.github.com/) is installed for large file meshes.
```
$ cd 'your catkin workspace src' && git clone --recurse https://github.com/shbang91/gazebo_scorpio_plugin.git
```

## Install Required Dependancies
Make sure [PnC Package](http://github.com/shbang91/PnC.git) is installed.
Clone the repo and checkout to Gazebo_Scorpio branch
```
$ mkdir build && cd build && cmake..
$ make -j
$ sudo make install
$ cd ../Addition/DataManager && mkdir build && cd build && cmake ..
$ make -j
$ sudo make install
```
## Example method to launch the Scorpio robot
In order to use the gazebo launch file, you have to expand the gazebo's model path

Add the following to your ~/.bashrc file:
```
$ export GAZEBO_MODEL_PATH=~/${catkin_ws}/src/:$GAZEBO_MODEL_PATH
$ export GAZEBO_PLUGIN_PATH=~/${catkin_ws}/devel/lib:$GAZEBO_PLUGIN_PATH
```
After compiling plugin package, Scorpio can be simulated in Gazebo via:
```
$ roslaunch gazebo_scorpio_plugin gazebo_scorpio.launch
```

Will's branch stuff:

I've not tested if things work without this step, but currently I recommend cloning and following the build instructions
for this repo: https://github.com/SyrianSpock/realsense_gazebo_plugin

EW notes on additional changes:
	-Gripper (TODO make branch in PnC reflecting these changes):
		1. Copy modified scorpio URDF compatible with PnC under models to 
		    PnC/RobotModel/Robot/Scorpio/Scorpio_Kin_robotiq_gripper.urdf
		2. In PnC/Scorpio/ScorpioInterface.cpp line 26, change RobotModel constructor
		    call to point to this new URDF
		3. Make sure you have the gazebo_grasp_plugin either from https://github.com/JenniferBuehler/gazebo-pkgs or included under third_party in https://github.com/Kinovarobotics/ros_kortex
	-Pour demo:
		- Change models such that they are small and light enough for gripper to pick up
			1. tr_jar - change scale to 0.022 0.022 0.0393701 and mass to 0.001 in sdf
			2. marbles - change mass to 0.0001
	-Vision concerns - not necessary for object detection to work in some capacity, but necessary for pour demo to work:
		- TODO - make more detailed and comprehensive list
		- plane masking
		- table height
		- Align sensors to center in realsense model
		- ...
