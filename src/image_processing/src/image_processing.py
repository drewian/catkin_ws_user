
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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def send_image(self):
    #try:
    #    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError as e:
    #    print(e)
    cv_image = cv2.imread("image.png", cv2.IMREAD_COLOR)
    #cv2.imshow(cv_image, cv2.IMREAD_COLOR)

    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 245
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    #find first pixel
    quadrants = [(0, 320, 0, 160), (0, 320, 160, 320), (0, 320, 320, 480), \
                (320, 640, 0, 160), (320, 640, 160, 320), (320, 640, 320, 480)]

    print(cv_image.shape)
    points = []
    for q in quadrants:
        #tmp_image = cv_image[q[0]:q[1], q[2]:q[3], :]
        for j in range(q[0], q[1]):
            found = False
            for i in range(q[2], q[3]):
                pixel = cv_image[i, j, 0]
                if pixel == 255:
                    points.append((i, j))
                    found = True
                    break
            if found: break

    # camera calibration
    points = [points[2], points[1], points[0], points[-1], points[-2], points[-3]]
    print(points)
    points = np.float32(points)
    realworld = np.float32([[0, 0, 0],[0, 20, 0],[0, 40, 0],[30, 0, 0],[30, 20, 0],[30, 40, 0]])
    K = np.float32([[614.1699, 0, 329.9491], [0, 614.9002, 237.2788], [0, 0, 1]])
    distortion = np.float32([0.1115, -0.1089, 0, 0])
    ret, rotation, translation = cv2.solvePnP(realworld, points, K, distortion)

    print("Rotation: ", rotation)
    print("Translation: ", translation)

    rodr, jacobi = cv2.Rodrigues(rotation)
    print("Rodrigues: ", rodr)

    rodr_inv = rodr.T
    inv = np.vstack([np.hstack([rodr_inv, -rodr_inv.dot(translation.reshape(3, 1))]), np.array([0, 0, 0, 1])])

    print ("Inverse homogeneous ", rodr_inv)

    sy = sqrt(rodr[0, 0] * rodr[0, 0] + rodr[1, 0] * rodr[1, 0])

    if sy < 1e-6:
        print("Rotation matrix is singular!")
    else:
        x = atan2(rodr[2, 1], rodr[2, 2])
        y = atan2(-rodr[2, 0], sy)
        z = atan2(rodr[1, 0], rodr[0, 0])
        print("X, Y, Y: ", x, y, z)



    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  #try:
  #  rospy.spin()
  #except KeyboardInterrupt:
  #  print("Shutting down")
  while True:
      print("Publishing image.")
      ic.send_image()
      sleep(15)
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
