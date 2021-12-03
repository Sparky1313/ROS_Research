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
### Starting Roscore
To run any of the software in ROS_Research, first open a terminal and run the following commands:
```
cd ~/catkin_ws
roscore
```

### Starting Gazebo and RViz
To use any of the software you will also need to open 2 more terminal windows based in `~/catkin_ws`.

Now run each of the following commands in a separate terminal in the order listed.
Make sure to let each command finish before starting the next one:
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml
```

#### Gazebo
`roslaunch turtlebot3_gazebo turtlebot3_house.launch` will open Gazebo, a program that provides a simulation environment for your simulated robot to run in. The command launches Gazebo using a TurtleBot3 and the map `turtlebot3_house.world` from the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` directory using the launch file `turtlebot3_house.launch`in the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch` directory.

After Gazebo loads it should look like this:

![Gazebo Terminal Image](/README_Photos/Gazebo_Terminal.png)

![Gazebo Startup Image](/README_Photos/Gazebo_Startup.png)

#### RViz
`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml` will open RViz, a program that allows you to *see* what your robot *sees* along with the path the robot is taking. The command launches RViz using the configuration in the launch file `turtlebot3_navigation.launch` in the directory `/opt/ros/melodic/share/turtlebot3_navigation/launch`. The `map_file` argument specifies what map for RViz to use. The map should be a map created by using a Slam Technique (discussed here (insert link to section)) of the world Gazebo is using for its simulation. For convenience, a map named `map1.yaml` of the `turtlebot3_house` has been included.

After RViz loads it should look like this:

![RViz Terminal Image](/README_Photos/RViz_Terminal.png)

![RViz Startup Image](/README_Photos/RViz_Startup.png)

After RViz loads you will need to manually position the robot's initial position to match where the robot is in Gazebo.

To do this, zoom out in RViz and find the coordinate that the robot matches up to. Keep track of this coordinate for later.

![RViz Zoom Out Image](/README_Photos/RViz_Zoom_Out.png)

Next, press the *2D Pose Estimate* button.

![RViz 2D Pose Estimate Image](/README_Photos/RViz_2D_Pose_Estimate.png)

Go back to the grid and click and hold on the coordinate that you found earlier. A green arrow should appear. The base of the arrow represents the base of the robot, and the head of the arrow represents the rotation of the robot. Try to draw the arrow so that it overlaps the grid line that runs straight to the top of the screen.

***TIP:** The farther out you drag your mouse, the more fine control you will have over the rotation.

Now, let go of the mouse and your robot should reposition according to where you clicked.

*If it didn't work, just try again. No other settings are changed when you reposition your robot.*

![RViz Robot Pose Initialized Image](/README_Photos/RViz_Robot_Pose_Initialized.png)

Once this is completed you are ready to run other ROS_Research software.

## Patrolling Package
### Listener.py

[listener.py](/src/patrolling/src/listener.py) is a simple script that allows you to create a custom path for your robot using RViz. This path can later be broadcast to the robot by running patrol.py ***Add in section link*** to make the robot patrol the path.

With Gazebo and RViz started up and initialized, open another terminal window.

Enter the following command:
`listener.py` ***Double Check Command and directory it needs to be run from.***
***Add Pictures***

After the listener.py is running, press the *2D Nav Goal* button. 

![RViz Robot Pose Initialized Image](/README_Photos/RViz_2D_Nav_Goal.png)

Just like you did for the *2D Pose Estimate* button in the [RViz section](https://github.com/Sparky1313/ROS_Research#rviz), pick a position and draw an arrow. [listener.py](/src/patrolling/src/listener.py) will then record that position in a file called [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). Your robot should then move to that position and orientation.

![RViz Robot Pose Initialized Image](/README_Photos/RViz_Nav_Goal_Sent.png)

When you are done just ^C in the terminal and the program will end.

**Be careful with listener.py!!! Whenever you start listener.py, it clears out the old coordinates in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). Be sure to save a copy of the waypoints if you want them later.*


### Patrol.py

[patrol.py](/src/patrolling/src/patrol.py) is a program that will read in the waypoints stored in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt) and broadcast to the robot so that the robot will move to them. The program uses [actionlib](http://wiki.ros.org/actionlib) and [move_base](http://wiki.ros.org/move_base) to accomplish this.

[patrol.py](/src/patrolling/src/patrol.py) also listens for interrupt tasks for the robot (interrupt tasks are not part of ROS, just a concept for the Patrolling Package). The interrupt tasks can stop the robot's travel its current waypoint and send a separate set of waypoints the robot needs to take for a task. After the robot completes the interrupt task path, it will resume traveling to waypoint it was going to before the interrupt.

There is only one interrupt task currently defined for the TurtleBot3 and that is a [doorbell interrupt](src/robot_tasks/src/doorbell_waypoints.txt). The task is just a set of waypoints in a .txt file just like [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). More tasks can be created and placed in [robot_tasks/src](src/robot_tasks/src/). To implement these in [patrol.py](/src/patrolling/src/patrol.py) just add the file path to `self.task_dict` in the `Robot_Task_Listener` class.


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
- Add in instruction to download Gazebo object database.




