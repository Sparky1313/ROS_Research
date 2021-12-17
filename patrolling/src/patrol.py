#!/usr/bin/env python

# Code modified from ROS Developers LIVE-Class #56: Make Your Robot Patrol An Area by The Construct 
# https://youtu.be/p-ZG6E-PZVA?t=3424

import rospy
import actionlib
# import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from threading import Thread, Lock
from std_msgs.msg import String
# import sys

waypoints = []

def read_waypoints_file(filename):
    """Reads in the waypoints from a file."""
    with open(filename, 'r') as file:
        waypoints = []

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
        
        return waypoints



class Patrol:
    """A class that sets the goals for a robot to move to."""
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.has_robot_task_interrupt = False
        self.is_executing_interrupt_routine = False

    def set_goal_to_point(self, position, orientation):
        """Sets the goals that a robot should travel to using MoveBaseGoal."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        self.client.send_goal(goal)
        
        client_result = ""

        # 3 is the success status for having made it to the waypoint.
        while client_result != 3:
            client_result = self.client.get_state()
            
            if self.has_robot_task_interrupt and not self.is_executing_interrupt_routine:
                self.client.cancel_all_goals()
                # rospy.loginfo("All goals cancelled.")
                self.is_executing_interrupt_routine = True

                # I believe this fixes the task interrupt getting overwritten before being scanned
                rospy.sleep(1)
                return False
        
        return True


            
class Robot_Task_Listener:
    """A class"""
    def __init__(self, patrol):
        # A dictionary that contains the file paths associated with different robot tasks.
        self.task_dict = {
            'doorbell': "src/robot_tasks/src/doorbell_waypoints.txt"
        }
        # The Patrol object this listener does callbacks for.
        self.patrol = patrol

    def callback(self, data):
        """The action that Robot_Task_Listener will take upon receiving a message."""
        # print(data)
        self.patrol.has_robot_task_interrupt = True
        # Generalize this eventually
        waypoints = read_waypoints_file(self.task_dict['doorbell'])
        # print(waypoints)
        rospy.loginfo("Robot task received.")
        
        for i, w in enumerate(waypoints):
            rospy.loginfo("Sending robot task waypoint %d", i)
            self.patrol.set_goal_to_point(w[0], w[1])
        
        self.patrol.is_executing_interrupt_routine = False
        self.patrol.has_robot_task_interrupt = False
        

    def listen(self):
        """Listens for robot tasks on the topic 'robot_tasks'. Invokes a specified callback when a message is received."""
        rospy.loginfo("Trying to listen for robot tasks...")
        rospy.Subscriber('robot_tasks', String, self.callback)
        rospy.loginfo("Listening for robot tasks.")



if __name__ == '__main__':
    rospy.loginfo("Reading waypoint list...")
    main_patrol_waypoint_file = "src/patrolling/src/patrol_waypoints.txt"
    waypoints = read_waypoints_file(main_patrol_waypoint_file)
    # print(waypoints)
    rospy.loginfo("Done reading waypoint list.")
    rospy.init_node('patrolling')

    try:
        patrol = Patrol()
        listener = Robot_Task_Listener(patrol)
        listener.listen()
        did_complete_last_point = True

        while not rospy.is_shutdown():
            for i, w in enumerate(waypoints):
                # If the robot receives a robot task between waypoints.
                # (Look into using a lock for this evenutally)
                while patrol.has_robot_task_interrupt: 
                    rospy.sleep(1)

                rospy.loginfo("Sending waypoint %d", i)
                did_complete_last_point = patrol.set_goal_to_point(w[0], w[1])

                while not did_complete_last_point:
                    # Wait for the robot task to complete.
                    if patrol.has_robot_task_interrupt:
                        rospy.sleep(1)
                    # Resends the last waypoint before the robot was interrupted by a robot task.
                    else:
                        rospy.loginfo("Sending waypoint %d", i)
                        did_complete_last_point = patrol.set_goal_to_point(w[0], w[1])

    except rospy.ROSInterruptException:
        rospy.logerr("Something went wrong when sending the waypoints.")