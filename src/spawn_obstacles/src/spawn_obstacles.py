#!/usr/bin/env python

from os import path, unsetenv
import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from nav_msgs.msg import Path
import math


def cleanup_models(model_set, delete_func):
    for item in model_set:
        delete_func(item)
    print "All models deleted"


def clean_input(input):
    input = input.lower()
    input = input.strip()
    return input


class Path_Listener:
    def __init__(self):
        # self.file = file
        self.task_dict = {
            'doorbell': "src/robot_tasks/src/doorbell_waypoints.txt"
        }
        self.path = Path()
        self.spawn_point = Point()
        self.has_path = False
        self.has_spawn_point = False

    def pick_point(self):
        print "picking point"
        num_path_waypoints = len(self.path.poses)
        spawn_point_pose_stamp_index = int(num_path_waypoints * 3 / 4)  # Tries to pick a point that is far enough away so that it will spawn by the time the robot gets to that position
        self.spawn_point = self.path.poses[spawn_point_pose_stamp_index].pose.position     # Look under nav_msgsand geometry_msgs to find these properties
        self.has_spawn_point = True

    def listen(self):
        print "listening"
        # rospy.init_node('robot_task_listener', anonymous=True)
        self.path = rospy.wait_for_message('move_base/DWAPlannerROS/global_plan', Path)
        # check to see if this message is listened to on 
        print "received message"
        self.has_path = True
        self.pick_point()

        # rospy.spin()
    
    def reset_info(self):
        self.path = Path()
        self.spawn_point = Point()
        self.has_path = False
        self.has_spawn_point = False


if __name__ == '__main__':
    GAZEBO_MODEL_PATH = 'src/gazebo_models'

    print("Waiting for gazebo services...")
    rospy.init_node("spawn_obstacles")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    path_listener = Path_Listener()

    with open("{}/beer/model.sdf".format(GAZEBO_MODEL_PATH), "r") as f:
        model_xml = f.read()

    thing = tf.transformations.quaternion_from_euler(0,0,0)
    print "{}".format(thing)
    orient = Quaternion(0, 0, 0, 1)
    # orient = Quaternion(tf.transformations.quaternion_from_euler(0,0,0))

    # for num in xrange(0,12):
    #     model_name = "product_{0}_0".format(num)
    #     print("Deleting model:%s", model_name)
    #     delete_model(model_name)

    # for num in xrange(0,12):
    #     bin_y   =   2.8 *   (num    /   6)  -   1.4 
    #     bin_x   =   0.5 *   (num    %   6)  -   1.5
    #     model_name   =   "product_{0}_0".format(num)
    #     print("Spawning model:%s", model_name)
    #     model_pose   =   Pose(Point(x=bin_x, y=bin_y,    z=2),   orient)
    #     spawn_model(model_name, model_xml, "", model_pose, "world")

    i = 1
    model_set = set()

    while not rospy.is_shutdown():
        is_valid_input = False
        user_input = raw_input("Spawn a new obstacle: Enter \'S\'\nDelete an obstacle: Enter \'D\'\nCleanup: Enter \'C\'\nCleanup and exit: Enter \'X\'\n")
        user_input = clean_input(user_input)
        
        if user_input == 's':
            path_listener.listen()
            model_name = "beer_{}".format(i)# string from last slash to period
            # x_coor = i
            # y_coor = i
            # find x and y somewhere within bounds of path
            while not path_listener.has_spawn_point:
                pass

            model_pose = Pose(Point(x=path_listener.spawn_point.x, y=path_listener.spawn_point.y, z=path_listener.spawn_point.z), orient)
            spawn_model(model_name, model_xml, "", model_pose, "world")
            model_set.add(model_name)
            path_listener.reset_info()
            i += 1
            print "{} added".format(model_name)
        elif user_input == 'd':
            while is_valid_input == False:
                user_input = raw_input("Enter the name of the obstacle you want to delete or enter \'R\' to return to the pevious menu:")
                user_input = clean_input(user_input)

                if user_input == 'r':
                    break
                elif user_input.isspace():
                    print "Only whitespace was detected."
                else:
                    if user_input in model_set:
                        is_valid_input = True

                        try:
                            delete_model(user_input)
                            print "{} was successfully deleted".format(user_input)
                        except:
                            print "Failed to delete {}".format(user_input)
                    else:
                        print "{} does not exist. Double-check the name you entered.".format(user_input)

        elif user_input == 'c':
            cleanup_models(model_set, delete_model)
        elif user_input == 'x':
            cleanup_models(model_set, delete_model)
            print "Exiting..."
            break
    



