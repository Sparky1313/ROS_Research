#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

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
    with open("src/patrolling/src/patrol_waypoints.txt", 'w') as file:
        listener = Listener(file)
        listener.listen()