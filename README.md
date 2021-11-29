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

If any issues arise try this command, although you shouldn't need it:
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
'echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc`








