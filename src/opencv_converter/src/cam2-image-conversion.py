#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("ImageConverter/cv_image", Image, queue_size=10)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("cam2/image_raw", Image, self.callback)
		self.phase_pub = rospy.Publisher("SAUVC/phase", Int8, queue_size=10)

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# INSERT OPENCV ALGORITHM HERE
		lower = np.array([0, 130, 153], dtype = "uint8")
		upper = np.array([100, 228, 255], dtype = "uint8")

		mask = cv2.inRange(cv_image, lower, upper)
		output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

		percentage = np.mean(output != cv_image) * 100	

		if percentage < 70:
			print("cam2: %4.3f%% Passed Gate!" % percentage)
			self.phase_pub.publish(1)
		else:
			print("cam2: %4.3f%%" %percentage)

		cv2.imshow("Image window", np.hstack([cv_image, output]))
		cv2.waitKey(10)

		# END OPENCV ALGORITHM HERE

		try:
		    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
		    print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter2', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
