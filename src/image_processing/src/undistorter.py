import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry import findRealPos
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
#from IPython import display

distortion = np.array([-0.2848041238667301, 0.0659206596895867, 0.0001416462376699293, -0.002084876269696945, 0]).reshape((5, 1))

cameraMatrix = np.array([366.8488601935177, 0, 331.7407554090113, 0, 366.7888272707089, 240.8568678888034, 0, 0, 1]).reshape((3, 3))

bridge = CvBridge()
skip_cluster_detection = False
dims = (640, 480)
scale_factor = 4
scaled_dims = (dims[0] // scale_factor, dims[1] // scale_factor)

def imgCallback(img_msg):
    global bridge, skip_cluster_detection, scaled_dims, scale_factor, realworld, cameraMatrix
    cv_img = bridge.compressed_imgmsg_to_cv2(img_msg)
    copy = cv_img
    cb, cg, cr = cv2.split(copy)
    #cv_img = cv2.resize(cv_img, scaled_dims)
    copy = cv2.undistort(copy, cameraMatrix, distortion)
    cv2.imshow("undist", copy)
    cv2.imshow("dist", cv_img)

    cv2.moveWindow("undist", 100, 100)
    cv2.moveWindow("dist", 900, 100)
    cv2.waitKey(0)

rospy.init_node("undistorter")
#rospy.Subscriber("/usb_cam/image_color/compressed", CompressedImage, imgCallback, queue_size=1)
rospy.Subscriber("/image_processing/image_raw", CompressedImage, imgCallback, queue_size=1)

rospy.spin()
