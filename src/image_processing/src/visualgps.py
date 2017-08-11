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
from geometry import findRealPos, rigid_transform_3D
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt


realworld = {
    "red": (3.57, 3.0),
    "blue": (3.57, 1.15),
    "purple": (2.33, 3.0),
    "green": (2.33, 1.15)
}

# blue center
color_pos_means = {
    "blue": (334, 227),
    "red": (173, 225),
    "green": (337, 112),
    "purple": (184, 118)}

distortion = np.array([-0.287348224330035, 0.05364856147008246,
                       0.00921819101079906, 0.003424786599010765, 0]).reshape((5, 1))
cameraMatrix = np.array([275.0386008874681, 0, 296.3819553629593, 0,
                         273.0746410330246, 215.4805927869714, 0, 0, 1]).reshape((3, 3))

bridge = CvBridge()

def imgCallback(img_msg):
    global bridge, realworld, cameraMatrix, color_pos_means, distortion
    cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
    cv_img = cv2.undistort(cv_img, cameraMatrix, distortion)

    cv2.imshow("gps", cv_img)
    cv2.moveWindow("gps", 100, 100)
    cv2.waitKey(50)



rospy.init("visual_gps", anonymous=True)

#rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, imgCallback, queue_size=1)
rospy.Subscriber("/image_processing/image_raw", CompressedImage, imgCallback, queue_size=1)
#rospy.Subscriber("/app/camera/rgb/image_raw/compressed", CompressedImage, imgCallback, queue_size=1)

rospy.spin()
