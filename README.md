# Background
This is a repository that I used to explore different topics in ROS.

I only included the files that ended up working.


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
```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```


## Getting TurtleBot3 Simulation Package
These instructions are from [Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).

Run these commands in the terminal:
```
cd ~/catkin_ws/src/
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

## Getting Gazebo_Models
[gazebo_models](https://github.com/osrf/gazebo_models) is a database of different Gazebo models that people can use. We will need them to spawn obstacles later on.

Run these commands in the terminal:
```
cd ~/catkin_ws/src/
git clone https://github.com/osrf/gazebo_models.git
```

More information on the database is available here: [http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot)


# Installing ROS_Research
At this point you should be able to install ROS_Research.

Run the command:
```
cd ~/catkin_ws/src/
git clone https://github.com/Sparky1313/ROS_Research.git
cd ~/catkin_ws && catkin_make
```

Next, run:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

When you are done, the bottom 3 lines of your `~/.bashrc` file should look like:
```
source /opt/ros/melodic/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
source ~/catkin_ws/devel/setup.bash
```


# Running Programs in ROS_Research
## Basic Commands When Starting any ROS_Research Software
### Starting Roscore
To run any of the software in ROS_Research, first open a terminal and run the following commands:
```
cd ~/catkin_ws
roscore
```

### Starting Gazebo and RViz
To use any of the software, you will also need to open 2 more terminal windows based in `~/catkin_ws`.

Now run each of the following commands in a separate terminal in the order listed.
Make sure to let `roslaunch turtlebot3_gazebo turtlebot3_house.launch` finish before starting the next one `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/map1.yaml`:
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/map1.yaml
```

#### Gazebo
`roslaunch turtlebot3_gazebo turtlebot3_house.launch` will open Gazebo, a program that provides a simulation environment for your simulated robot to run in. The command launches Gazebo using a TurtleBot3 and the map `turtlebot3_house.world` from the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` directory using the launch file `turtlebot3_house.launch`in the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch` directory.

After Gazebo loads it should look like this:

![Gazebo Terminal Image](/README_Photos/Gazebo_Terminal.png)

![Gazebo Startup Image](/README_Photos/Gazebo_Startup.png)


#### RViz
`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/map1.yaml` will open RViz, a program that allows you to *see* what your robot *sees* along with the path the robot is taking. The command launches RViz using the configuration in the launch file `turtlebot3_navigation.launch` in the directory `/opt/ros/melodic/share/turtlebot3_navigation/launch`. The `map_file` argument specifies what map for RViz to use. The map should be a map of the world Gazebo is using for its simulation. This map should be created using a [SLAM technique](https://github.com/Sparky1313/ROS_Research#creating-a-custom-slam-map). For convenience, a map named [map1.pgm](src/map1.pgm) (along with its yaml file [map1.yaml](src/map1.yaml)) of `turtlebot3_house` has been included.

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

Once this is completed, you are ready to run other ROS_Research software.


## Creating a Custom SLAM Map
*This section is optional.*

SLAM stands for Simultaneous Localization and Mapping. I won't go into detail about the theory here, but there are plenty of resources online that explain it.

To use Gazebo with RViz, you need a map of a Gazebo world that can work with RViz. As stated in the previous section, there is a map already included.

If you would like to make your own map using SLAM, you can follow this [tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/).

## patrolling Package
### listener.py
#### listener.py Description
[listener.py](/src/patrolling/src/listener.py) is a simple script that allows you to create a custom path for your robot using RViz. This path can later be broadcast to the robot by running [patrol.py](https://github.com/Sparky1313/ROS_Research#patrolpy) to make the robot patrol the path.

#### Running listener.py
With Gazebo and RViz started up and initialized, open another terminal window from `~/catkin_ws`.

Enter the following command:
```
rosrun patrolling listener.py
```

After the listener.py is running, press the *2D Nav Goal* button. 

![RViz Robot Pose Initialized Image](/README_Photos/RViz_2D_Nav_Goal.png)

Just like you did for the *2D Pose Estimate* button in the [RViz section](https://github.com/Sparky1313/ROS_Research#rviz), pick a position and draw an arrow. [listener.py](/src/patrolling/src/listener.py) will then record that position in a file called [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). Your robot should then move to that position and orientation.

![RViz Robot Pose Initialized Image](/README_Photos/RViz_Nav_Goal_Sent.png)

When you are done, just ^C in the terminal and the program will end.

**Be careful with listener.py!!! Whenever you start listener.py, it clears out the old coordinates in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). Be sure to save a copy of the waypoints if you want them later.*

**By default there are already waypoints in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt), so you can run [patrol.py](/src/patrolling/src/patrol.py) without having to use [listener.py](/src/patrolling/src/listener.py) to record your own waypoints .*


### patrol.py
#### patrol.py Description
[patrol.py](/src/patrolling/src/patrol.py) is a program that will read in the waypoints stored in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt) and broadcast to the robot so that the robot will move to them. The program uses [actionlib](http://wiki.ros.org/actionlib) and [move_base](http://wiki.ros.org/move_base) to accomplish this.

[patrol.py](/src/patrolling/src/patrol.py) also listens for robot tasks for the robot (robot tasks are not part of ROS, just a concept for the Patrolling package). The robot tasks can stop the robot's travel at its current waypoint and send a separate set of waypoints the robot needs to go to for a task. After the robot completes the robot task path, it will resume traveling to waypoint it was going to before the interrupt.

There is only one robot task currently defined for the TurtleBot3 and that is a [doorbell task](src/robot_tasks/src/doorbell_waypoints.txt). The task is just a set of waypoints in a .txt file just like [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt). More tasks can be created and placed in [robot_tasks/src](src/robot_tasks/src/). To implement these in [patrol.py](/src/patrolling/src/patrol.py), just add the file path to `self.task_dict` in the `Robot_Task_Listener` class.

**By default there are already waypoints in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt), so you can run [patrol.py](/src/patrolling/src/patrol.py) without having to use [listener.py](/src/patrolling/src/listener.py) to record your own waypoints .*


#### Running patrol.py
To run [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt), ensure that Gazebo and RViz are started up and initialized. 

Open another terminal window from `~/catkin_ws` and enter the following command:
```
rosrun patrolling patrol.py
```

The robot should start patrolling the waypoints in [patrol_waypoints.txt](/src/patrolling/src/patrol_waypoints.txt) and repeat the path indefinitely.

![Patrolling](/README_Photos/Patrolling.png)


#### Sending a Robot Task
To run send a robot task, ensure that Gazebo and RViz are started up and initialized. Then make sure that 
```
rosrun patrolling patrol.py
```
has been run.

Open another terminal window from `~/catkin_ws` and enter the following command:
```
rostopic pub /robot_tasks std_msgs/String "task_name"
```
Replace `task_name` with name of your task that you placed in `self.task_dict` in the `Robot_Task_Listener` class.

To run the included doorbell task, enter the following command:
```
rostopic pub /robot_tasks std_msgs/String "doorbell"
```
This will send the robot to "answer" the door. It will then resume traveling to last waypoint before it was interrupted.

![Doorbell Robot Task](/README_Photos/Robot_Task.png)


## spawn_obstacles Package
### spawn_obstacles.py
#### spawn_obstacles.py Description
[spawn_obstacles.py](/src/spawn_obstacles/src/spawn_obstacles.py) is a program that will spawn obstacles in front of the robot as it travels to a waypoint so that it will have to maneuver around them. This creates a dynamic environment to test against.

When obstacles are spawned, they will be assigned a specific id/name that can be used to reference them for deletion. By default, the object will be a beer bottle. You can modify [spawn_obstacles.py](/src/spawn_obstacles/src/spawn_obstacles.py) to use other objects from ```gazebo_models``` as obstacles.

![Obstacle Name](/README_Photos/Obstacle_Name.png)


#### Running spawn_obstacles.py
With Gazebo, RViz, and [patrol.py](/src/patrolling/src/patrol.py) started up and initialized, open another terminal window from `~/catkin_ws`.

Run the command:
```
rosrun spawn_obstacles spawn_obstacles.py
```

This will open a command line menu:

![Spawn Obstacles Main Menu](/README_Photos/Spawn_Obstacle_Main_Menu.png)

From there, you have the option to spawn an obstacle, delete a specific obstacle, cleanup (delete all) obstacles, or cleanup and exit.


#### Spawn an Obstacle
Enter 'S' to spawn an obstacle:

![Spawn Obstacle 3](/README_Photos/Spawn_Obstacle_3.png)

![Spawn Obstacle 1](/README_Photos/Spawn_Obstacle_1.png)

![Spawn Obstacle 2](/README_Photos/Spawn_Obstacle_2.png)

An obstacle will be prevented from spawning if it would spawn too close to the robot or a waypoint. The following message will appear if this occurs:

![Spawn Obstacle Cannot Spawn](/README_Photos/Spawn_Obstacle_Cannot_Spawn.png)


#### Delete an Obstacle
Enter 'D' to delete an obstacle. The following menu will appear:

![Delete Obstacle Menu](/README_Photos/Delete_Obstacle_Menu.png)

Enter the obstacle name that you want to delete and the obstacle will be deleted if it exists:

![Delete Obstacle 2](/README_Photos/Delete_Obstacle_2.png)

![Delete Obstacle 1](/README_Photos/Delete_Obstacle_1.png)


#### Cleanup (Delete All) Obstacles
Enter 'C' to cleanup (delete all) the obstacles:

![Cleanup Obstacles 2](/README_Photos/Cleanup_Obstacles_2.png)

![Cleanup Obstacles 1](/README_Photos/Cleanup_Obstacles_1.png)


#### Cleanup and Exit
Enter 'X' to cleanup and exit the program:

![Cleanup and Exit](/README_Photos/Cleanup_and_Exit.png)