#!/usr/bin/env python

# Code modified from ROS Developers LIVE-Class #56: Make Your Robot Patrol An Area by The Construct 
# https://youtu.be/p-ZG6E-PZVA?t=3424

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
            elif "pose" in line:
                waypoint = []
                position = []
                orientation = []

                file.readline()
                
                for x in range(3):
                    line = file.readline()
                    text = line.split(": ")
                    position.append(float(text[1]))
                
                waypoint.append(position)
                file.readline()
                
                for x in range(4):
                    line = file.readline()
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

                
class Listener:
    def __init__(self, file):
        self.file = file

    def callback(self, data):
        print(data)
        data = str(data)
        self.file.write(data + "\n")

    def listen(self):
        print("trying to listen")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.callback)
        print("listening")
        rospy.spin()


if __name__ == '__main__':
    print("Reading waypoint list...")
    read_waypoints_file()
    print(waypoints)
    print("Done reading waypoint list")

    rospy.init_node('patrolling')
    try:
        # p = Patrol()
        # while not rospy.is_shutdown():
        #     for i, w in enumerate(waypoints):
        #         rospy.loginfo("Sending waypoint %d - %s", i, w[0])
        #         p.set_goal_to_point(w[1])

        patrol = Patrol()
        while not rospy.is_shutdown():
            for i, w in enumerate(waypoints):
                rospy.loginfo("Sending waypoint %d", i)
                patrol.set_goal_to_point(w[0], w[1])
    except rospy.ROSInterruptException:
        rospy.logerr("Something went wrong when sending the waypoints")