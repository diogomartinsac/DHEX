

# DHEX

### Lyapunov Controller

The package for this controller is dhex_lyapunov_contrller which is organized as:
```
|-action
|--LyapunovController.action
|-config
|--lyapunov_controller_local.yaml
|-launch
|--dhex_lyapunov_controller.launch
|-include
|--LyapunovController.h
|-src
|--LyapunovController.cpp
|--lyapunov_controller_node.cpp
|--pose_command_node.cpp
```

The action folder contains the msg type created to be used, which has a goal pose as an input, a feedback of the current pose error and a boolean result to show if the control was able to guide the robot the the correct pose.

The config folder has the parameters used by the LyapunovController code, which basically reads the odometry data, receives the goal pose command and applies the LyapunovControl. Is this config file the k1 and k2 values are set, along with the distance and agle tolerance.

lyapunov_controller_node is the node that executes lyapunov control.
pose_command_node is the node that sends the command to the lyapunov controller. This node need to be called with four arguments, which are de x, y and theta goal, as well as the constant velocity used by the robot.

### How to execute
For testing the controller use the following commands in three different terminals:
```
roslaunch dhex_description simulation.launch 
roslaunch dhex_lyapunov_controller dhex_lyapunov_controller.launch 
rosrun dhex_lyapunov_controller pose_command 5 5 0 0.5
```

The last command can be manipulated to change the setpoint. To get the pose error use the `rostopic echo /lyapunov_controller/feedback` in another terminal.

### Simulation Video

In this vídeo (https://youtu.be/sYGCKwRxdqU) the setpoint given was (x, y, theta) =  (5, 4, 0).
A crazy behaviour close to the goal was indentified but not its root cause. It appers that for some angles the normalization is not being correctly done. 

### Supported versions

This work was developed and tested under [Ubuntu 18.04 LTS](https://ubuntu.com/#download) using [ROS Melodic Morenia] and
under [Ubuntu 20.04 LTS](https://ubuntu.com/#download) using [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu). This software may work on other platforms.


### How to install

Inside the catkin_workspace folder, which can be created by following this [ROS tutorial](http://wiki.ros.org/ROS/Tutorials), clone our repository or download it as a ZIP file and extract it there.

Then, inside the catkin_workspace/src folder, build the workspace with `$ catkin_make`
