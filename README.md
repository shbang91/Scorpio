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
$ export GAZEBO_MODEL_PATH=~/${your catkin ws}/src/:$GAZEBO_MODEL_PATH
```
After compiling plugin package, Scorpio can be simulated in Gazebo via:
```
$ roslaunch gazebo_scorpio_plugin gazebo_scorpio.launch
```
