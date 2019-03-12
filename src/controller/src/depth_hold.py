#!/usr/bin/env python

import rospy
from std_msgs.msgs import UInt16, Float32, String
from mavros_msgs.msg import Mavlink
from struct import pack, unpack

def listener():
	rospy.init_node('depth_listener', anonymous=True)
	pub_pressure = rospy.Publisher("depth_listener/pressure_diff", Float32, queue_size=10)
	pub_throttle = rospy.Publisher("BlueRov2/rc_channel3/set_pwm", UInt16, queue_size=10)
	rospy.Subscriber("/mavlink/from", Mavlink, callback)
	rospy.spin()

def callback(data):
	# msgid for SCALED_PRESSURE2 is 137 (Bar30 Sensor)
	# msgid for SCALED_PRESSURE is 29 (Pixhawk internal pressure sensor)
	# https://mavlink.io/en/messages/common.html
	if data.msgid == 137:
		rospy.loginfo(rospy.get_caller_id() + "Package: %s", data)
		p = pack("QQ", *data.payload64)
		time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)
		# Pressure is in hectopascal (hPa)
		# 100 hPa = 1.0197 m of H20
		# 50 hPa = 0.5 m of H20
		if press_diff > 70:
			pub_throttle.publish(1550)
		elif press_diff > 53:
			pub_throttle.publish(1510)
		elif press_diff < 30:
			pub_throttle.publish(1450)
		elif press_diff < 47:
			pub_throttle.publish(1490)
		pub_pressure.publish(press_diff)

if __name__ == "__main__":
	listener()
