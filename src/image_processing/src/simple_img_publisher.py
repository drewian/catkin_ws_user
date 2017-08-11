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

# from __future__ import print_function

class simple_pub:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/image_raw", CompressedImage, queue_size=1)


    self.bridge = CvBridge()
    #self.image_sub = rospycentercriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)

    self.img = cv2.imread("image4.png", cv2.IMREAD_COLOR)
    self.bridge = CvBridge()
    self.img = self.bridge.cv2_to_compressed_imgmsg(self.img)#, "bgr8")


  def run(self):
    while True:
      self.image_pub.publish(self.img)
      print("Published image")
      sleep(0.2)


if __name__ == "__main__":
    rospy.init_node("simple_pub", anonymous=True)
    sb = simple_pub()
    sb.run()



