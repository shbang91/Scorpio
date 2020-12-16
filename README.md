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

## Realsense Camera

Will's branch stuff:

I've not tested if things work without this step, but currently I recommend cloning and following the build instructions
for this repo: https://github.com/SyrianSpock/realsense_gazebo_plugin

## EW additional changes for pour demo

## ADE

1. Get Git Permissions:
You'll need to be given proper permissions to access the git repo. Go to the HRI Lab Gitlab and create a new account. Then add your ssh key(s) to you account, and request access to the ade project.

2. Clone the Git Repo
```
$ git clone ssh://git@hrilab.tufts.edu:22222/ade/ade
```

3. Install required dependencies
check java version:
```
$ java -version
```
if java 1.8 is not installed:
```
$ apt install openjdk-8-jdk
```
If more than one version is installed, select java 1.8
```
$ sudo update-alternatives --config java
```

4. compile using Ant Targets
```
$ ./ant
```
^ could be out of date depending on version of ADE you have, if this doesn't work
instead use:
```
$ apt install ant
$ ant
```
To build only specific components necessary for the scorpio, rather than everything, run:
```
$ ant scorpio-demo
$ ant vision
```
(If you do not have ROSADE or vision already set up then you will need to follow the 

	following instructions before building scorpio-demo which has rosade-core as a 

	dependency or vision which needs vision to be present on your machine)

## ROSADE 

ROSADE is necessary in order to communicate with the scorpio in gazebo

1. Install ROS
Follow the instructions from the ros website https://wiki.ros.org/ to get a working
environment.

	This has only been tested so far using ROS Melodic on Ubuntu 18.04, but I believe Kinetic should work as well.

2. Install ROSJAVA
Don't install rosjava via apt, rather get the code using
```
$ git clone https://github.com/evankrause/rosjava_core.git
```
Add ROSJAVA and ADE directories to your ROS_PACKAGE_PATH, either one-time in the command
line or added to your \~/.bashrc file, run
```
export ROS_PACKAGE_PATH=<ADE>:<ROSJAVA>:$ROS_PACKAGE_PATH
``` 
if added to .bashrc, make sure to 'source \~/.bashrc' before moving on
Then in the ROSJAVA directory, build with
```
$ ./gradlew install
```
Now you should be able to run 'ant rosade-core' sucessfully

## Vision

TODO: Are there publicly available instructions for setting up vision?

Getting native vision set up can be a pain and copying the entire setup instructions here
   seems redundant. For now please reach out to any of the group at TR and we will find
   a way to get you access to the setup instructions

Note: only special instructions in terms of setup here is that vision should be 
		configured to enable ROS

\*\*Note: The current branch of vision desired is the thinking_robots_temp branch

## Working Robotiq Gripper

TODO: make branch in PnC reflecting these changes

1. Make minor changes in PnC to update URDF 

Copy modified scorpio URDF compatible with PnC under models to 
PnC/RobotModel/Robot/Scorpio/Scorpio_Kin_robotiq_gripper.urdf

Then in PnC/Scorpio/ScorpioInterface.cpp line 26, change RobotModel constructor
call to point to this new URDF

2. Install gazebo grasp fix plugin

Make sure you have the gazebo_grasp_plugin either from 
https://github.com/JenniferBuehler/gazebo-pkgs or included under third_party in
https://github.com/Kinovarobotics/ros_kortex

I personally have it included as part of the ros_kortex pacakge, but I assume the usual
build pattern after cloning the standalone repo should work:
```
$ cd gazebo-pkgs/gazebo_grasp_plugin
$ mkdir build
$ cd build
$ cmake ..
$ make -jnproc
$ sudo make install
```
TODO: at this point not sure make install is necessary or if adding it to the
  GAZEBO_PACKAGE_PATH is better/necessary. If you're following these instructions
  and find that one works and the other doesn't, please update the README or let me know

## Models

All modified models utilized within the various worlds are under the models
  subdirectory, it seems that the above addition to the gazebo model path doesn't
  catch these models. Please additionally run

```
$ export GAZEBO_MODEL_PATH=~/${catkin_ws}/src/gazebo_scorpio_plugin/models:$GAZEBO_MODEL_PATH
```
Or add this line to your \~/.bashrc file and 'source \~/.bashrc'

## Vision Config

(Work In Progess: please reach out to me as you encounter more problems and I can
	update this list to make it easier for future users)

At the time of writing this (12/16) vision should be configured such that everything is
  set up for the pour demo in sim to work, but in case you want to edit or create new
  worlds yourself or the state of the repos inevitably change, I will provide a list of 
  both necessary and helpful config changes to get vision working.

1. Camera transform (necessary) - TODO: give more detailed instructions
	
In the corresponding launch file that you are using, the line defining the camera 
  transform, e.g. (this transform is the one for the current state of the demo)

```
<node pkg="tf" type="static_transform_publisher" name="camera_tf_publisher"     args="0 -1.3 1.7 0.9238915 0 0 -0.3826545 /world camera 100"/>
```
must contain the correct transform (xyz pose and xyzw quaternion orientation) of the
  camera in simulation relative to the base of the robot. In this specific example the 
  camera is 1.7 meters above the floor (where the robot base connects), -1.3 meters on 
  the y-axis (as defined by gazebo), and facing the robot (whose base is at 0,0,0) with 
  a downward angle of 45 degrees.

You can find this transform in whatever way you see fit, the position should be easy
  just looking at the pose of the camera model in gazebo. In terms of the orientation, 
  I personally found that it was easiest to just visualize the relationship between the 
  different coordinate frames, manually create the rotation matrix, and subsequently 
  convert that to a quaternion. The main thing to be wary of that took me a while to 
  figure out is that, while the position elements of the transform are relative to 
  gazebo's coordinate frame (or at least seem to be), the coordinate frame of the camera 
  off of which to base the rotation matrix is not the same as that of gazebo, and is 
  seemingly arbitrary. In order to visualize what these axes are, go to the vision GUI 
  when running the simulation and ADE, click on the 'show Depth' box on the bottom of 
  the GUI, and left click and drag on the black window that pops up to visualize the 
  depth cloud. The camera axes should be floating somewhere in the scene, which you can 
  use the colors of to match to the gazebo axes.

To extend this to the real world, you will as of now have to hand measure these things,
  the relative position of the camera with a tape measure and the orientation of the
  camera with a protractor (or through iterative adjustments using vision results as a
  reference which would require disabling some functionality and generally be tedious - 
  you can reach out to me if you're having trouble getting this working and cannot
  measure things well). I would ideally limit the orientation of the camera to some 
  right angle about the z axis so you mostly just have to measure and consider the 
  angle below parallel the camera is facing to make it easier, but with 
  the correct transform any orientation should work.

2. Table Height Param (necessary):

There is no automatic decision as of now in vision and no great config setup for
  the plane detection necessary to get cloud segmentation and therefore object
  detection to work. In vision/native/imgproc/PlaneProcessor.cpp in the two calls
  to SegmentPlane() and SegmentPlaneGivenCoeffs(), the additional argument for table
  height must match the height of the surface, I believe, above the base of the robot.
  If these are not set properly, then vision will look for either the wrong plane or one
  that doesn't exist and will not work properly.
(In the case of the sim this value is currently 1.0)

3. Plane color masking (optional):

Due to cloud smoothing, clustering outliers, and noise in the sensor data, oftentimes
  errant points from the plane cloud are included in the segmented object clouds either
  at the base of or floating behind the object, which messes with location and dimension
  calculations for detected objects. In terms of demos this is an issue when inconsistent
  sensor noise can make the detected object location vary and grasp behavior be 
  inconsistent.

In vision/native/data/imgProcs/plane/plane_mask.xml there should be different existing
  configs for other surfaces that we have already used. For use with the table model
  in simulation just comment out whatever existing predicate block is there and uncomment
  the corresponding block labeled as gazebo sim table if necessary.

If you want no masking to occur, uncomment the top block. If you want to add a new block
  for a new surface, feel free to do so. The best way to find appropriate values for each
  parameter is to run vision with the surface in the scene (ideally under consistent
  lighting conditions, feel free to add a small buffer to the values to account for 
  differences in lighting) and the Vision GUI/'Camera Control Panel'. Once here, navigate
  to the 'Blob Parameters' tab, press 'Add Color Range', press 'Define Color' - a window
  with a snapshot of the current scene should pop up. At this point click in three (or 
  more) locations to define the vertices of the area you want to grab the color range of.
  Press 'OK' in the window and then 'OK' again under the 'Blob Sizes' box. On the left
  the color range should be defined (rmin,gmin,bmin to rmax,gmax,bmax) which you can copy
  the values of for use in the param file.
(I believe these values are RGB, if they are ordered differently I apologize and please
  let me know to update this)

The two main issues with this method are that it will not work if the surface is 
  multicolored (or the range will just be so big that everything is masked out),
  but even if multicolored the masking will work for whatever region the mask is
  defined for - just not across all the different regions. Also if the objects are
  the same color as the surface then they will be masked out and won't be detected as
  objects.

4. Various other params (optional):

TODO: expand and elaborate on this list as issues arise, no one setting is best for all
		scenarios

- Max height off table for object detection: in 
	vision/native/point_clouds/PCLFunctions.hpp, line 829 the argument height defines the
	max acceptable height for a point above the plane surface to still be considered
	a candidate object point. This is helpful to avoid e.g. the arm being considered an
	object, but if set too low can cut off the tops of objects you may care about.

- Min blob size for object detection: in vision/native/detector/ClusterDetector.cpp
    constructor, value within object_cluster_min_size. To avoid sensor noise, irrelevant
    objects, or uneven parts of the plane to be picked up as objects, increase this value
    so they are filtered out during clustering. Similarly, if your object is smaller than
    the threshold then it won't be detected as well

- Clustering sensitivity: also in vision/native/detector/ClusterDetector.cpp
    constructor, depending on the amount of noise in the point cloud and scene/object
    of interest, tightening or relaxing the cluster tolerance can lead to better results

- Sensor data smoothing/camera params: Not very familiar with these params, but they
    can be found in vision/native/capture/CaptureRealSense.cpp (or whatever 
    corresponding capture method you are using)
    (For sim in CaptureRos.cpp, but the camera parameters are pulled from a ROS topic)

## Misc

### Object Teaching/Saving Models for the GlobalFeatureValidator

As of now there is no great streamlined method to save point clouds of specific objects
that you want the system to remember between different executions, although teaching
objects within a single execution is easier. I will first explain how object teaching
works and then what you would have to do to get these models stored in a more permanent
fashion.

Object Teaching: So within a single execution of the system, you can use the 
  corresponding commands (through the SimSpeech GUI or however) to teach the system
  about an object - 'Do you see the object' and then 'This object is a \_'. This will
  temporarily save the detected object cloud and update a temporary xml file to include
  that cloud for the duration of the demo. Note that if multiple objects are detected at
  the time of saying 'This object is a \_', vision will select one randomly and consider
  it whatever you say, so clear the scene of other objects when you are doing this. This
  method however will delete the temporary cloud and updated xml file after some time.

(TODO: I'm actually not sure if one or both of the models themselves or the 
       xml.auto_save get deleted after time or not, I've just always manually updated
       the base files and copied the clouds. It's possible this next step isn't even
       necessary other than to keep the config clean and not accidentally add more 
       models)

To more permanently save models and add them to the config, follow these additional
  steps. When you undergo the above steps, the object cloud will be saved in the file
  (lets use 'do you see the jar' for example) 
  vision/native/data/imgProcs/global_feature/pcd/jar/jar_view_<#>.pcd. If you want to
  keep a copy of this file, then do so and copy it to another file or rename it. I
  believe if you keep doing iterations of 'this is the jar', even across runs, that
  it will save additional files incrementing the <#> of the file, but to be safe I
  copy them elsewhere. Additionally, an auto_save version of your config file will be
  created. Personally, I prefer to update the config I am using myself between runs
  if I need to rather than use the auto_save version just to make sure nothing has
  accidentally been added or removed. In this case, under 
  vision/native/data/imgProcs/global_feature/{config_filename}.xml follow the examples
  to add or remove models and their corresponding descriptors as you see fit. The more
  different models/predicates that are contained in the file, the slower model matching
  will occur and the greater chance there will be for error, but generally extra
  predicates aren't that big of a deal.

(for sim the current config_filename is ericglobal_features.xml)

To clarify what the actual effect of object teaching is, is that after doing the above
  steps for example, anytime you reference the 'jar' it will look for an object matching
  that model and consider that to be the object you are referring to (e.g. when you say
  pick up the jar).


### Offsets in primitives due to vision limitations

I just want to note somewhere that due to sensor noise and camera callibration/transform
not being exact, as well as inherent issues in vision with the object clouds not being
complete based on the view angle, that it is unlikely for some primitives to work as is
without some fine tuning. If you try to recycle primitives across different objects
or different vision configuration it is likely that execution will be slightly off. In
this case you will have to update some of the primitive offsets or contact one of us to
help you do so. We will work on making this process more general as well as updating 
vision algorithms or adding heuristics to make it so this is not usually the case, but
for now it will be necessary.

### Logging (both vision and ADE)

So this section could be extremely long, but for now I will just point you to the two
  files that determine the logging levels of different components on the ADE side and
  different vision processes on the native side.

Native: in vision/native/data/logging/log4cxx.txt, uncomment (delete the #) any lines
	you want to recieve logging messages for

ADE: First in ade/anthome/perUserFiles/{username}.properties add the line 
     'logging.config.file=config/logging/{username}.xml' - or whatever existing logging
     setup you want to copy. Then in that file that you give the path to, you can set
     the root level of the logger as well as component specific overrides by 
     adding/uncommenting the corresponding line.

(where username is your user on your specific system, e.g. /home/{username})

To see what log statements should actually be printed during execution, you'd have to
 look into the source code, and can add whatever you want as long as it's above the
 level set in the xml file.

## Run Demo

After following all the prior instructions and having all of ADE/ROSADE, Vision, and
  the Gazebo_scorpio_plugin along with all depenencies built and configured correctly, 
  we should be ready to run the demo.

In two separate terminals run
```
roslaunch gazebo_scorpio_plugin scorpio_gazebo_camera_gripper.launch world_name:=scorpio_pour.world

```
to start the simulation and spawn in the scorpio and camera models, and 
```
ant launch -Dargs=config/perUserFiles/mfawn/ScorpioPourDemo.config
```
(or ant launch -Dargs=config/perUserFiles/mfawn/ScorpioPourDemoFirebase.config for 
  remote communication)

to start up ADE and vision.

If all initializes correctly and runs, a bunch of GUIs should pop up for various 
  components. For the purposes of runnning the demo, the only GUI you need to worry
  about is the SimSpeech GUI which contains a list of pre-defined phrases and a chat box.
  In order the run the demo, either find or manually type in the following phrases:

'Do you see the jar' - wait for response (will take longer because of initial plane seg)

'Do you see the box' - wait for response

'the jar is now open' - wait for response

'I want marbles in the jar' - at this point the arm should start executing the pour

(optional when the pour is complete) 'stop pouring' - the arm should reorient the jar