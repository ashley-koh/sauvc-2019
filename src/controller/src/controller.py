#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt16, Bool

# Global Publishers
pub_mode = rospy.Publisher("BlueRov2/mode/set", String, queue_size=10)
pub_arm = rospy.Publisher("BlueRov2/arm", Bool, queue_size=10)
pub_throttle = rospy.Publisher("BlueRov2/rc_channel3/set_pwm", UInt16, queue_size=10)
pub_forward = rospy.Publisher("BlueRov2/rc_channel5/set_pwm", UInt16, queue_size=10)

def controller():
	# Subscribe to timer node
	sub_timer = rospy.Subscriber("timer_sec/time", Int32, callback)

	# Init Node
	rospy.init_node('controller', anonymous=True)

	# Wait for next second
	rospy.spin()

# Callback for control (based on time)
def callback(time): 
	if time.data < 20:
		pass
	elif time.data < 40:
		pass


if __name__ == "__main__":
	controller()