

# Diffrential A* algorithm : Project 3
---

## Overview

C++ project to explore node using A* algorithm with diffrential constraint


## Coordianate Axis

The coordinate axis is changed from the center of the map to the bottom left corner.

## Dependencies

This project uses the following packages:
1. ROS Kinetic
2. Ubuntu 16.04
3. Packages Dependencies:

 * Turtlebot3 ROS packages
 * roscpp
 * rospy
 * std_msgs
 * geometry_msgs



 Install ROS turtlebot3 dependent packages:

  ```sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers ros-kinetic-hector-mapping```

  For Turtlebot3 packages follow the following steps:
  ```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
catkin_make
```


- ## Sourcing
Open your bashrc file
```
gedit ~/.bashrc
```
Type the following in your bashrc file:

  Replace ```<IP>``` with your systems IP address
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<IP>:11311
export ROS_HOSTNAME=<IP>
export TURTLEBOT3_MODEL=waffle
 ```
save and close the bashrc file and source it
```source ~/.bashrc```


- ## Build Instruction

unzip the files to this location </catkin_ws/src/>

and then run following command in the terminal
```
cd ~/catkin_ws/src/
cd ..
catkin_make
```


## Run Instruction


open terminal and type the following command :
```
roslaunch frontier_exploration_turtlebot diffrential.launch
```

Once the exploration is completed launch gazebo using this command line in new terminal:
```
roslaunch frontier_exploration_turtlebot gazebo.launch
```

Press enter on the terminal to see simulation on gazebo
