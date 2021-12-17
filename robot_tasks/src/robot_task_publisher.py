#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def publish_task():
    pub = rospy.Publisher('robot_tasks', String, queue_size=10)
    rospy.init_node('robot_tasks', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass