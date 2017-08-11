import numpy as np
from math import atan, cos, sin, pi, atan2


def findRealPos2(imagePoints, modelPoints, mmPerPixel=1):
    numPoints = len(imagePoints)

    colors = imagePoints.keys()
    imagePoints = [imagePoints[color] for color in colors[:numPoints]]
    modelPoints = [modelPoints[color] for color in colors[:numPoints]]

    imagePoints = mmPerPixel * np.array(imagePoints).reshape(numPoints, 2)
    modelPoints = np.array(modelPoints).reshape(numPoints, 2)

    imgCenter = np.sum(imagePoints, axis=0) / numPoints
    modelCenter = np.sum(modelPoints, axis=0) / numPoints

    fstSum = 0
    sndSum = 0
    for i in range(numPoints):
        a = imagePoints[i, 0] * modelPoints[i, 1]
        b = imagePoints[i, 1] * modelPoints[i, 0]
        fstSum += a - b
        a = imagePoints[i, 0] * modelPoints[i, 0]
        b = imagePoints[i, 1] * modelPoints[i, 1]
        sndSum += a + b

    yaw = atan2(fstSum, sndSum)  - pi/2
    xrot = np.array([cos(yaw), -sin(yaw)]).reshape((1, 2))
    yrot = np.array([sin(yaw), cos(yaw)]).reshape((1, 2))

    tx = modelCenter[0] - xrot.dot(imgCenter)
    ty = modelCenter[1] - yrot.dot(imgCenter)

    t = np.array([tx, ty])

    return yaw, t


# Python impl of algo described in http://nghiaho.com/?page_id=671
def findRealPos(imagePoints, modelPoints, mmPerPixel):
    numPoints = len(imagePoints)

    colors = imagePoints.keys()
    imagePoints = [imagePoints[color] + (0,) for color in colors[:numPoints]]
    modelPoints = [modelPoints[color] + (0,) for color in colors[:numPoints]]

    imagePoints = mmPerPixel * np.array(imagePoints).reshape(numPoints, 3)
    modelPoints = np.array(modelPoints).reshape(numPoints, 3)

    imgCenter = np.sum(imagePoints, axis=0) / numPoints
    modelCenter = np.sum(modelPoints, axis=0) / numPoints

    print("Image points: ", imagePoints)

    # Rotate modelPoints into imagePoints!
    #    H = np.zeros((3, 3))
    #    for i in range(numPoints):
    #        H += (modelPoints[i] - modelCenter).dot((imagePoints[i] - imgCenter).T) #.dot((modelPoints[i] - modelCenter).T)
    #        #H += (imagePoints[i] - imgCenter).dot((modelPoints[i] - modelCenter).T) #.dot((modelPoints[i] - modelCenter).T)

    modelCentered = modelPoints - modelCenter
    imageCentered = imagePoints - imgCenter

    print("Image centroid: ", imgCenter)
    H = modelCentered.T.dot(imageCentered)

    U, S, V = np.linalg.svd(H)
    R =  V.T.dot(U.T)
    det = np.linalg.det(R)
    if det < 0:
        R[2, :] *= -1 #R[2, :]

    tvec =  -R.dot(modelCenter) + imgCenter
    #tvec =  -R.dot(imgCenter) + modelCenter
    #tvec = R.dot(tvec)

    return R, tvec, imgCenter, modelCenter

from numpy import *
from math import sqrt

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA).dot(BB)

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R.dot(centroid_A.T) + centroid_B.T

    print("Another Translation: ", t)

    return R, t

