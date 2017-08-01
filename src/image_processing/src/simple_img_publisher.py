#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
from time import sleep
from math import atan2, sqrt
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

# from __future__ import print_function

class simple_pub:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/image_raw",Image, queue_size=1)


    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)

    self.img = cv2.imread("image.png", cv2.IMREAD_COLOR)
    self.bridge = CvBridge()
    self.img = self.bridge.cv2_to_imgmsg(self.img, "rgb8")


  def run(self):
    while True:
      self.image_pub.publish(self.img)
      sleep(5)


if __name__ == "__main__":
    rospy.init_node("simple_pub", anonymous=True)
    sb = simple_pub()
    sb.run()








