import cv2
import pyrealsense as pyrs
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import roslib
import rospy


def callback(data):
	print("got some data")
	img = br.compressed_imgmsg_to_cv2(data)
	cv2.imwrite('messigray.png',img)
	cv2.waitKey(10)

br = CvBridge()

rospy.Subscriber('picture', CompressedImage, callback)
rospy.init_node('get_picture', anonymous=True)
rospy.spin()