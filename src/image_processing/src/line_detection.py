# #!/usr/bin/env python
# import roslib
# # roslib.load_manifest('my_package')
# import sys
# import rospy
# import cv2
# import numpy as np
# from std_msgs.msg import String
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge, CvBridgeError
# import matplotlib
# from time import sleep
# from math import atan2, sqrt
# #matplotlib.use('Agg')
# from matplotlib import pyplot as plt
# #from IPython import display

# bridge = CvBridge()
# skip_cluster_detection = False
# dims = (640, 480)
# scale_factor = 4
# scaled_dims = (dims[0] // scale_factor, dims[1] // scale_factor)

# def imgCallback():
# 	global bridge, skip_cluster_detection, scaled_dims, scale_factor
# 	#cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
# 	cv_img = cv2.imread("lane1.png",cv2.IMREAD_COLOR)
# 	cb, cg, cr = cv2.split(cv_img)

# 	cv_img = cv2.resize(cv_img, scaled_dims)
# 	cv2.imshow("lidec", copy)

# 	#cv_img-cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)

import cv2
import numpy as np

def imgCallback():
	cv_img = cv2.imread("lane1.png",cv2.IMREAD_COLOR)
	cv_img = cv2.resize(cv_img, scaled_dims)
    cv2.imshow("lidec", copy)
	gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
	# define range of blue color in HSV
	lower_blue = np.array([110,50,50])
	upper_blue = np.array([130,255,255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(cv_img,cv_img, mask= mask)
	cv2.imshow('gray',gray)
	cv2.imshow('hsv',hsv)
	# cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
# cv2.destroyAllWindows()