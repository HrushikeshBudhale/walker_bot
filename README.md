
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# walker_bot
---

### Overview
 This repository walker_bot ROS package with one talker_bot_node. This node provides the functionality for the turtlebot3 to move around and avoid obstacles using the readings from the lidar scan data.

 The launch file provides additional functionality to record the rosbag while launching the node.

### Dependencies
- ROS version:
    - Melodic
- Packages:
    - catkin
    - roscpp
    - geometry_msgs
    - sensor_msgs
    - turtlebot3_gazebo


### Assumptions
 It is assumed that the user has installed ROS Melodic full desktop version and has created a catkin workspace folder with src folder inside it.

### Steps to install
 Install turtlebot3_gazebo if not installed using following command
```
sudo apt-get install ros-melodic-turtlebot3
sudo apt-get install ros-melodic-turtlebot3-simulations
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```
 
 To install the walker_bot package, enter following commands in the 'src/' directory of your catkin workspace
```
git clone git@github.com:HrushikeshBudhale/walker_bot.git
cd ..
catkin_make --pkg walker_bot
source devel/setup.bash
```


### Launch walker_bot
 In your catkin workspace, enter following command to launch walker_bot_node. 
```
source devel/setup.bash
roslaunch walker_bot walker_bot_node.launch
```
 On successful launch, a gazebo environment will open with turtlebot3 moving inside the environment while avoiding the collision with obstacles ahead.

### Launch walker_bot with rosbag record
 Enter following command to start recording a rosbag on launching the node.
```
roslaunch walker_bot walker_bot_node.launch record_bag:=true
```
 The generated rosbag contains all the topics except topics related to camera and scan. This rosbag will be saved in the results directory of this package.
 Prerecorded rosbag can be found [here](https://drive.google.com/file/d/1QrrBjUbBJmiWtDHl4RwzKZ0AYmLVcjFt/view?usp=sharing).