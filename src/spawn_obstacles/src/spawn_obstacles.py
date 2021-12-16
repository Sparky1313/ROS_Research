#!/usr/bin/env python

# from logging import root
# from os import path, unsetenv
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetLinkState, GetModelState
from geometry_msgs.msg import *
from nav_msgs.msg import Path
import math
import xml.etree.ElementTree as ET

# Manually taken from /turtlebot3_waffle.urdf.xacro specifically /opt/ros/melodic/share/turtlebot3_description/turtlebot3_waffle.urdf.xacro
# The length of one side of a turtlebot3
TURTLEBOT3_SIDE_LENGTH = 0.266

# The turtlebot's typical pivot is in the rear of the turtlebot so the swing radius is larger than the side length.
# In addition, it will backup and scooch forward some when adjusting. Therefore, we multiply the length by 2 to
# give a margin of error for the robot to rotate and adjust.
TURTLEBOT3_SAFE_BASE_SWING_RADIUS = TURTLEBOT3_SIDE_LENGTH * 2 

def delete_all_obstacles(obstacle_set, delete_func):
    """Deletes all of the obstacles the user has created."""
    for item in obstacle_set:
        delete_func(item)
    print "All obstacles deleted"


def clean_user_input(input):
    """Cleans the user input."""
    input = input.lower()
    input = input.strip()
    return input


def parse_model_radius(xml_file):
    """Parses the model's radius from its sdf file."""
    tree = ET.parse(xml_file)
    root = tree.getroot()
    # print "{}".format(root)
    # This needs to be edited depending on the model.
    radius_element = root.find("./model/link/collision/geometry/cylinder/radius")
    radius_val = float(radius_element.text)
    return radius_val
    

class Path_Listener:
    """A class that listens to paths broadcasted on 'move_base/DWAPlannerROS/global_plan' and picks a possible point to spawn an obstacle"""
    def __init__(self):
        self.path = Path()
        self.spawn_point = Point()
        self.path_endpoint = Point()
        self.has_path = False
        self.has_spawn_point = False


    def pick_point(self):
        """Picks a possible to point on the robot's current path to spawn an obstacle."""
        print "Picking a possible point for spawning an obstacle..."
        num_path_waypoints = len(self.path.poses)
        spawn_point_pose_stamp_index = int(num_path_waypoints / 2)  # Tries to pick a point that is far enough away so that it will spawn by the time the robot gets to that position
        self.spawn_point = self.path.poses[spawn_point_pose_stamp_index].pose.position  # Look under nav_msgs and geometry_msgs to find these properties
        self.path_endpoint= self.path.poses[num_path_waypoints - 1].pose.position
        self.has_spawn_point = True


    def listen(self):
        """Waits for a message from 'move_base/DWAPlannerROS/global_plan' that details the robot's current path."""
        print "Listening..."
        self.path = rospy.wait_for_message('move_base/DWAPlannerROS/global_plan', Path)
        # print "Received a path."
        self.has_path = True
        self.pick_point()

    
    def reset_info(self):
        """Resets the Path_Listener's properties to default."""
        self.path = Path()
        self.spawn_point = Point()
        self.path_endpoint = Point()
        self.has_path = False
        self.has_spawn_point = False


if __name__ == '__main__':
    GAZEBO_MODEL_PATH = 'src/gazebo_models'

    # Wait for nodes and services
    print "Waiting for gazebo services..."
    rospy.init_node("spawn_obstacles")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print "All services ready."

    # Create the necessary service proxies and a Path_Listener
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    get_model_state = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    get_link_state = rospy.ServiceProxy("gazebo/get_link_state", GetLinkState)
    path_listener = Path_Listener()

    # Read the model's sdf file
    with open("{}/beer/model.sdf".format(GAZEBO_MODEL_PATH), "r") as f:
        model_xml = f.read()
    
    beer_radius = parse_model_radius("{}/beer/model.sdf".format(GAZEBO_MODEL_PATH))

    ###########3
    # check to see if this one works beer_radius = parse_model_radius(model_xml)
    ###########

    # print "{}".format(beer_radius)
    # print "{}".format(get_model_state("turtlebot3", None).pose.position)
    # print "{}".format(get_link_state("turtlebot3/base_link", None).link_state.pose.position)
    # print "{}".format(model_xml)
    
    # orientation = Quaternion(tf.transformations.quaternion_from_euler(0,0,0)) # For some reason there were issues with this

    # Create default orientation for obstacle
    orientation = Quaternion(0, 0, 0, 1)
    # A variable that keeps track of how many obstacles have been spawned.
    # The number is concatenated to the obstacle's number to give it a unique identifier.
    obstacle_num = 1
    # A set that contains all of the different obstacles created.
    obstacle_set = set()

    # Run forever
    while not rospy.is_shutdown():
        is_valid_input = False
        # Main menu options
        user_input = raw_input("Spawn a new obstacle: Enter \'S\'\nDelete an obstacle: Enter \'D\'\nCleanup: Enter \'C\'\nCleanup and exit: Enter \'X\'\n")
        user_input = clean_user_input(user_input)
        
        # User selected to spawn an obstacle
        if user_input == 's':
            path_listener.listen()
            obstacle_name = "beer_{}".format(obstacle_num)# string from last slash to period
            # x_coor = obstacle_num
            # y_coor = obstacle_num

            # Wait for a spawn point
            while not path_listener.has_spawn_point:
                pass

            obstacle_pose = Pose(Point(x=path_listener.spawn_point.x, y=path_listener.spawn_point.y, z=path_listener.spawn_point.z), orientation)
            robot_pose = get_model_state("turtlebot3", None)

            # double check that this works from the original then get rid of abs value

            # Calculating Pythagorean values for the triangle between the obstacle spawn point and the robot's position.
            a_squared = abs(obstacle_pose.position.x - robot_pose.pose.position.x) ** 2
            b_squared = abs(obstacle_pose.position.y - robot_pose.pose.position.y) ** 2
            c_squared = math.sqrt(a_squared + b_squared)
            distance_between_robot_and_spawn_obstacle = c_squared - beer_radius

            # Calculating Pythagorean values for the triangle between the path endpoint and the obstacle spawn point.
            a_squared = abs(path_listener.path_endpoint.x - obstacle_pose.position.x) ** 2
            b_squared = abs(path_listener.path_endpoint.y - obstacle_pose.position.y) ** 2
            c_squared = math.sqrt(a_squared + b_squared)
            distance_between_path_endpoint_and_spawn_obstacle = c_squared - beer_radius

            # distance_between_robot_and_spawn_obstacle = math.sqrt(abs(obstacle_pose.position.x - robot_pose.pose.position.x) ** 2 + abs(obstacle_pose.position.y - robot_pose.pose.position.y) ** 2) - beer_radius
            # distance_between_path_endpoint_and_spawn_obstacle = math.sqrt(abs(path_listener.path_endpoint.x - obstacle_pose.position.x) ** 2 + abs(path_listener.path_endpoint.y - obstacle_pose.position.y) ** 2) - beer_radius

            if distance_between_robot_and_spawn_obstacle < TURTLEBOT3_SAFE_BASE_SWING_RADIUS or distance_between_path_endpoint_and_spawn_obstacle < TURTLEBOT3_SAFE_BASE_SWING_RADIUS:
                print "Cannot spawn the obstacle. Obstacle is too close to robot's current position or a path endpoint."
                # print "{}".format(path_listener.path_endpoint.x)
                # print "{}".format(path_listener.path_endpoint.y)
                # print "{}".format(obstacle_pose.position.x)
                # print "{}".format(obstacle_pose.position.y)
                # print "{}".format(TURTLEBOT3_SAFE_BASE_SWING_RADIUS)
                # print "{}".format(distance_between_path_endpoint_and_spawn_obstacle)
                # print "{}".format(path_listener.path)
                # print "{}".format(len(path_listener.path.poses))
                continue

            spawn_model(obstacle_name, model_xml, "", obstacle_pose, "world")
            obstacle_set.add(obstacle_name)
            path_listener.reset_info()
            obstacle_num += 1
            print "{} added".format(obstacle_name)

            # print "{}".format(obstacle_pose)
            # print "{}".format(robot_pose)
            # print "{}".format(distance_between_robot_and_spawn_obstacle)

        # User selected to delete an obstacle
        elif user_input == 'd':
            while is_valid_input == False:
                user_input = raw_input("Enter the name of the obstacle you want to delete or enter \'R\' to return to the pevious menu:")
                user_input = clean_user_input(user_input)

                if user_input == 'r':
                    break
                elif user_input.isspace():
                    print "Only whitespace was detected."
                else:
                    if user_input in obstacle_set:
                        is_valid_input = True

                        try:
                            delete_model(user_input)
                            print "{} was successfully deleted".format(user_input)
                        except:
                            print "Failed to delete {}".format(user_input)
                    else:
                        print "{} does not exist. Doublecheck the name you entered.".format(user_input)

        elif user_input == 'c':
            delete_all_obstacles(obstacle_set, delete_model)
        elif user_input == 'x':
            delete_all_obstacles(obstacle_set, delete_model)
            print "Exiting..."
            break
    



