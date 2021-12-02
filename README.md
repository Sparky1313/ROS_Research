# Background

This is a repository that I used to learn about ROS and to explore different topics in ROS. Not all of the exploration produced tangible results, but some did.

# Setting Up ROS From the Beginning
This section will teach you how to setup ROS to be able to use this repository.

If you have installed all of the necessary packages, then you may skip this section.

## Installing ROS
To use this repository, you will want to install [Ubuntu 18](https://releases.ubuntu.com/18.04/).

Next you will need to install [Ros Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). Follow the instructions for installation provided by the ROS wiki.
- For step 1.4 Installation follow the Desktop-Full Install

![Installation Image](/README_Photos/Installation_Step_1_4.png)

## Configuring the ROS Environment
For initial configuration of your ROS environment, follow this [guide](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
- Under the heading **3. Create a ROS Workspace** you will see an optional instruction about setting up ROS with Python 3. **DON'T DO IT!** Everything should be running Python 2.

![Python 3 Dependency](/README_Photos/Python_3_Dependency.png)

# Installing Necessary TurtleBot3 Packages
[ROBOTIS e-Manual for TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) will be a valuable resource should you run into any issues in this section.

## Getting Basic TurtleBot3 Packages
These instructions are from the [Quick Start Guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

Run these commands in the terminal:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
```

If any issues arise, try this command (although you shouldn't need it):
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
ros-melodic-rosserial-server ros-melodic-rosserial-client \
ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
ros-melodic-compressed-image-transport ros-melodic-rqt* \
ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

Next, run:
`echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc`

## Getting TurtleBot3 Simulation Package
These instructions are from [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

Run these commands in the terminal:
```
cd ~/catkin_ws/src/
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

# Installing ROS_Research
At this point you should be able to install ROS_Research.

Run the command:
```
cd ~/catkin_ws/src/
git clone https://github.com/Sparky1313/ROS_Research.git
cd ~/catkin_ws && catkin_make
```

# Creating a custom SLAM Map

# Running Programs in ROS_Research

## Basic Commands When Starting any ROS_Research Software
To run any of the software in ROS_Research, first open a terminal and run the following commands:
```
cd ~/catkin_ws
roscore
```

To use any of the software you will also need to open 2 more terminal windows based in `~/catkin_ws`.

Now run each of the following commands in a separate terminal in the order listed.
Make sure to let each command finish before starting the next one:
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml
```

`roslaunch turtlebot3_gazebo turtlebot3_house.launch` will open Gazebo, a program that provides a simulation environment for your simulated robot to run. The command launches Gazebo using a TurtleBot3 and the map `turtlebot3_house.world` from the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` directory using the launch file `turtlebot3_house.launch`in the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch` directory.

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml` will open RViz, a program that allows you to "see" what your robot sees along with the path the robot is taking. The command launches RViz using the configuration in the launch file `turtlebot3_navigation.launch` in the directory `/opt/ros/melodic/share/turtlebot3_navigation/launch`.


Next, open 5 more terminal windows based in that directory.

Now run each of the following commands in a separate terminal in the order listed.
Make sure to let each command finish before starting the next one:
```
roscore
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml
rosrun patrolling patrol.py
rosrun spawn_obstacles spawn_obstacles.py
```



NOTES TO ME:
- Test if you need to source /devel/setup.bash or if that is leftover from an experiment




