#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
#from IPython import display

def imgCallback():
	global bridge, skip_cluster_detection, scaled_dims, scale_factor, realworld, cameraMatrix
	#cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
	#cv_img = cv2.imread("center.png",cv2.IMREAD_COLOR)
	cv_img = cv2.imread("image3.png",cv2.IMREAD_COLOR)
	#gps_screenshot_09.08.2017.png
	copy = cv_img

	cv2.imshow("gps", copy)

	cv2.moveWindow("gps", 100, 100)


	lower_blue=np.array([105,80,80])
	upper_blue=np.array([130,255,255])
	lower_green=np.array([45,80,80])
	upper_green=np.array([75,255,255])
	lower_red=np.array([0,80,80])
	upper_red=np.array([30,255,255])
	lower_purple=np.array([135,80,80])
	upper_purple=np.array([165,255,255])
	hsv = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
	
	mask_blue=cv2.inRange(hsv,lower_blue,upper_blue)
	mask_green=cv2.inRange(hsv,lower_green,upper_green)
	mask_red=cv2.inRange(hsv,lower_red,upper_red)
	mask_purple=cv2.inRange(hsv,lower_purple,upper_purple)
	cv2.imshow('blue',mask_blue)
	cv2.waitKey(0)
	cv2.imshow('green',mask_green)
	cv2.waitKey(0)
	cv2.imshow('red',mask_red)
	cv2.waitKey(0)
	cv2.imshow('purple',mask_purple)
	cv2.waitKey(0)

imgCallback()

