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
	#cv_img = cv2.resize(cv_img, scaled_dims)
	#cv2.imshow("lidec", copy)
	gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
	hls = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HLS)
	lower_white_rgb = np.array([200,200,200])
	upper_white_rgb = np.array([245,245,245])
	white_rgb = cv2.inRange(cv_img,lower_white_rgb,upper_white_rgb)
	lower_white=np.array([0,0,0])
	upper_white=np.array([0,0,255])
	mask_white=cv2.inRange(hsv,lower_white,upper_white)

	lower_blue = np.array([110,50,50])
	upper_blue = np.array([130,255,255])
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	res = cv2.bitwise_and(cv_img,cv_img, mask= white_rgb)
	
	lower_green = np.array([50,100,100])
	upper_green = np.array([70,255,255])
	mask_green = cv2.inRange(hsv,lower_green,upper_green)
	res_green = cv2.bitwise_and(cv_img,cv_img,mask= mask_green)


	#red = np.uint8([[[255,0,0]]])
	#hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
	#print(hsv_red)
	lower_red=np.array([110,100,100])
	upper_red=np.array([130,255,255])
	mask_red=cv2.inRange(hsv,lower_red,upper_red)
	res_red=cv2.bitwise_and(cv_img,cv_img,mask=mask_red)

	#white = np.uint8([[[255,255,255]]])
	#hsv_white = cv2.cvtColor(white,cv2.COLOR_BGR2HSV)
	#print(hsv_white)
	lower_white = np.array([0,0,100])
	upper_white = np.array([10,0,230])
	mask_white = cv2.inRange(hsv, lower_white, upper_white)
	res_white = cv2.bitwise_and(cv_img,cv_img,mask= mask_white)
	
	
	cv2.imshow('gray',gray)
	cv2.imshow('hsv',hsv)
	#cv2.imshow('mask_green',mask_green)
	#cv2.imshow('res_green',res_green)
	#cv2.imshow('mask_red',mask_red)
	#cv2.imshow('res_red',res_red)
	cv2.imshow('mask',mask)
	#cv2.imshow('mask_white',mask_white)
	cv2.imshow('res',res)
	cv2.imshow('white_rgb',white_rgb)
	#cv2.imshow('res_white',res_white)
	cv2.imshow('hls',hls)
	cv2.imshow('mask_white',mask_white)
	cv2.waitKey(0)
imgCallback()
# cv2.destroyAllWindows()
