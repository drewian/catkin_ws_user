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
from sklearn import linear_model, datasets


def imgCallback():
	leftRansac=[]
	rightRansac=[]
	leftX=[]
	leftY=[]
	rightX=[]
	rightY=[]
	kernel=np.ones((5,5),np.uint8)
	cv_img = cv2.imread("themiddlelane.png",cv2.IMREAD_COLOR)
	#cv_img = cv2.resize(cv_img, scaled_dims)
	#cv2.imshow("lidec", copy)
	gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
	hsv = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HSV)
	hls = cv2.cvtColor(cv_img,cv2.COLOR_BGR2HLS)
	lower_white_rgb = np.array([130,130,130])
	upper_white_rgb = np.array([255,255,255])
	white_rgb = cv2.inRange(cv_img,lower_white_rgb,upper_white_rgb)
	img_ero_dil = cv2.erode(white_rgb, kernel, iterations=1)
	img_ero_dil = cv2.dilate(img_ero_dil, kernel, iterations=1)
	cv2.imshow('ero_dil',img_ero_dil)

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
	#cv2.imshow('mask',mask)
	#cv2.imshow('mask_white',mask_white)
	#cv2.imshow('res',res)
	cv2.imshow('white_rgb',white_rgb)
	#cv2.imshow('res_white',res_white)
	cv2.imshow('hls',hls)
	#cv2.imshow('mask_white',mask_white)
	cv2.waitKey(0)
	ly=479
	ry=479
	for y in range(white_rgb.shape[0]):
		for x in range(white_rgb.shape[1]/2):
			if len(leftRansac)==10:
				break
			if white_rgb[ly-y,x]!=0:
				leftRansac.append((ly-y,x))
				ly-=10
				break

	for y in range(white_rgb.shape[0]):
		for x in range(white_rgb.shape[1]/2):
			if len(rightRansac)==10:
				break
			if white_rgb[ry-y,639-x]!=0:
				rightRansac.append((ry-y,639-x))
				ry-=10
				break

	print(leftRansac)
	print(rightRansac)

	if len(leftRansac) > 1:
		for a in range (len(leftRansac)):
			leftY.append(leftRansac[a][0])

		#np.array(leftY).reshape((3,1))
		print(leftY)


		for a in range (len(leftRansac)):
			leftX.append(leftRansac[a][1])

		#np.array(leftX).reshape((3,1))
		print(leftX)


		model_leftRansac = linear_model.RANSACRegressor(linear_model.LinearRegression())
		try:
			#model_ransac.fit(Xs, ys)
			model_leftransac.fit(leftY,leftX)
			print("Coeffs: ", model_ransac.estimator_.coef_)
		except:
			print("NOT ENOUGH POINTS!")

	if len(rightRansac) > 1:
		for a in range (len(rightRansac)):
			rightY.append(rightRansac[a][0])

		#np.array(rightX).reshape((3,1))
		print(rightY)
		
		for a in range (len(rightRansac)):
			rightX.append(rightRansac[a][1])

		#np.array(rightX).reshape((3,1))
		print(rightX)

		model_rightRansac = linear_model.RANSACRegressor(linear_model.LinearRegression())
		try:
			#model_ransac.fit(Xs, ys)
			model_rightRansac.fit(rightY, rightX)
			print("Coeffs: ", model_ransac.estimator_.coef_)
		except:
			print("NOT ENOUGH POINTS!")


			
			

	
imgCallback()
# cv2.destroyAllWindows()
