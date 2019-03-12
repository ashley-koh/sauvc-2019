#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int8, UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

	def __init__(self):
		self.movespeed = 50
		self.moveleft = 1500 + self.movespeed
		self.moveright = 1500 - self.movespeed
		self.lateral_pub = rospy.Publisher("BlueRov2/rc_channel4/set_pwm", UInt16, queue_size=10)
		self.image_pub = rospy.Publisher("ImageConverter/cv_image", Image, queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("cv_camera/image_raw", Image, self.callback)
		#self.phase_pub = rospy.Publisher("SAUVC/phase", Int8, queue_size=10)
		
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# INSERT OPENCV ALGORITHM HERE
		lower = np.array([0, 110, 110], dtype = "uint8")
		upper = np.array([102, 255, 255], dtype = "uint8")

		mask = cv2.inRange(cv_image, lower, upper)
		output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

		grey_image = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

		ret, thresh = cv2.threshold(grey_image, 127, 155, 0)

		M = cv2.moments(thresh)
		
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			cX, cY = 0, 0

		cv2.circle(output, (cX, cY), 5, (255, 255, 255), -1)
		cv2.putText(output, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		cv2.circle(output, (320, cY), 5, (255, 255, 255), -1)
		cv2.putText(output, "center", (295, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


		if cX > 200 and cX < 390:
			print("stay straight")
			self.lateral_pub.publish(1500)

		elif cX < 200:
			print("move left")
			self.lateral_pub.publish(self.moveleft)
			
		elif cX > 390:
			print("move right")
			self.lateral_pub.publish(self.moveright)
			

		cv2.imshow("Image window", np.hstack([cv_image, output]))
		cv2.waitKey(20)

		# END OPENCV ALGORITHM HERE

		try:
		    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
		    print(e)

def main(args):
    rospy.init_node('opencv', anonymous=False)
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("OpenCV Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
