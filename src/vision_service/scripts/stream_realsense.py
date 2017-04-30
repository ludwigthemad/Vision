import cv2
import pyrealsense as pyrs
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import roslib
import rospy
import pdb
import time

from cv_bridge import CvBridge

def callback(data):
	
	print("Taking a picture")
	print("sleeping for 2 seconds")
	time.sleep(2)
	pub.publish(msg)

## start pyrealsense service
pyrs.start()
br = CvBridge()

#Image Size (define size of image)
x_pixel = 640
y_pixel = 480

# resize for faster processing
resize_factor = 1

#init realsense device
dev = pyrs.Device(device_id = 0, streams = [pyrs.ColourStream(width = x_pixel, height = y_pixel, fps = 30), pyrs.DepthStream()])

pub = rospy.Publisher('picture', CompressedImage, queue_size=10)
rospy.Subscriber("trigger", Bool, callback)
rospy.init_node('send_picture', anonymous=True)

while True:
    # Get Frame from Realsense
	dev.wait_for_frame()
	# color image	    
	col = cv2.cvtColor(dev.colour, cv2.COLOR_RGB2BGR)
	#depth images
	dep = dev.depth*  dev.depth_scale * 1000

	#resize images for faster processing with resize_factor
	img = cv2.resize(col, (int(resize_factor*x_pixel),int(resize_factor*y_pixel)))
	
	d_img = cv2.resize(dep, (int(resize_factor*x_pixel),int(resize_factor*y_pixel)))
	d_img = cv2.applyColorMap(dep.astype(np.uint8), cv2.COLORMAP_RAINBOW)

	# show image and continue

	cv2.imshow("Camera stream", img)
	msg = br.cv2_to_compressed_imgmsg(img)
	# pdb.set_trace()
	cv2.waitKey(10)

cv2.destroyAllWindows() 
