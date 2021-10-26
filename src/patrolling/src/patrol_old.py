#!/usr/bin/env python

# Code modified from ROS Developers LIVE-Class #56: Make Your Robot Patrol An Area by The Construct 
# https://youtu.be/p-ZG6E-PZVA?t=3424

# Make sure that all dependencies are included in package.xml

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from threading import Thread, Lock
from std_msgs.msg import String
import sys

# waypoint = [
#     ['one', ()],
#     ['two', ()],
#     ['three', ()]
# ]
# waypoint_position = []
# waypoint_orientation = []
waypoints = []

class Patrol:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def set_goal_to_point(self, position, orientation):
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.position.x = point[0]
        # goal.target_pose.position.y = point[1]
        # quaternion = tf.transformations.quarternion_from_euler(0.0, 0.0, point[2])
        # goal.target_pose.pose.orientation.x = quaternion[0]
        # goal.target_pose.pose.orientation.y = quaternion[1]
        # goal.target_pose.pose.orientation.z = quaternion[2]
        # goal.target_pose.pose.orientation.w = quaternion[3]

        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

def read_waypoints_file():
    with open("src/patrolling/src/patrol_waypoints.txt", 'r') as file:
        while True:
            line = file.readline()
            
            if not line:
                break

            # When the pose section of the text file is reached
            elif "pose" in line:
                waypoint = []
                position = []
                orientation = []

                # Skip the position header
                file.readline()
                
                # Read the x, y, and z coordinates
                for x in range(3):
                    line = file.readline()
                    
                    # Skip the label of the coordinate and grab only the value
                    text = line.split(": ")
                    position.append(float(text[1]))
                
                waypoint.append(position)

                # Skip the orientation header
                file.readline()
                
                # Read the x, y, z, and w coordinates
                for x in range(4):
                    line = file.readline()

                    # Skip the label of the coordinate and grab only the value
                    text = line.split(": ")
                    orientation.append(float(text[1]))
                
                waypoint.append(orientation)
                waypoints.append(waypoint)

                # if "position:" in line:
                #     position = []

                #     for x in range(3):
                #         line = file.readline()
                #         print(line)
                #         text = line.split(": ")
                #         position.append(float(text[1]))

                #     waypoint.append(position)
                # elif "orientation:" in line:
                #     orientation = []

                #     for x in range(4):
                #         line = file.readline()
                #         text = line.split(": ")
                #         orientation.append(float(text[1]))

                #     waypoint.append(orientation)

                
class Robot_Task_Listener:
    def __init__(self, lock):
        # self.file = file
        self.task_dict = {
            'doorbell': "src/robot_tasks/src/doorbell_waypoints.txt"
        }
        self.lock = lock

    def callback(self, data):
        print(data)
        

    def listen(self):
        self.lock.acquire()
        print("trying to listen")
        # rospy.init_node('robot_task_listener', anonymous=True)
        rospy.Subscriber('robot_tasks', String, self.callback)
        print("listening")
        sys.stdout.flush()
        self.lock.release()
        # rospy.spin()


if __name__ == '__main__':
    lock = Lock()

    lock.acquire()
    rospy.loginfo("Reading waypoint list...")
    read_waypoints_file()
    print(waypoints)
    print("Done reading waypoint list")
    # sys.stdout.flush()
    lock.release()

    rospy.init_node('patrolling')
    try:
        # p = Patrol()
        # while not rospy.is_shutdown():
        #     for i, w in enumerate(waypoints):
        #         rospy.loginfo("Sending waypoint %d - %s", i, w[0])
        #         p.set_goal_to_point(w[1])

        
        patrol = Patrol()
        listener = Robot_Task_Listener(lock)
        listen_thread = Thread(target=listener.listen)
        listen_thread.start()

        while not rospy.is_shutdown():
            for i, w in enumerate(waypoints):
                lock.acquire()
                rospy.loginfo("Sending waypoint %d", i)
                lock.release()
                patrol.set_goal_to_point(w[0], w[1])
    except rospy.ROSInterruptException:
        rospy.logerr("Something went wrong when sending the waypoints")