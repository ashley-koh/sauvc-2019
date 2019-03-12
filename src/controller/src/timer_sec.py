#!/usr/bin/env python

# Init Node:    timer_sec
# Publish to:   timer_sec/time
# Subscribe to: None
# Description:
#   Publishes the number of seconds that have passed since initializing the node

import rospy
from std_msgs.msg import Int32

seconds = 0

def timer_sec():
    timer_pub = rospy.Publisher("timer_sec/time", Int32, queue_size=10)
    rospy.init_node('timer_sec', anonymous=True) 
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        timer_pub.publish(seconds)
        rospy.loginfo(seconds)
        seconds++
        rate.sleep()

if __name__ == "__main__":
    try:
        timer_sec()
    except rospy.ROSInterruptException:
        pass