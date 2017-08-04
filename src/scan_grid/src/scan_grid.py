#!/usr/bin/env python

# --- imports ---
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from math import sin, cos
from sklearn import linear_model, datasets
import numpy as np

# --- definitions ---
def resetGrid():
    global occupancy_grid

    # set all values to "FREE"
    for i in range(len(occupancy_grid.data)):
        occupancy_grid.data[i] = 0
        

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y,value=100):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = value

def default_values():
    global occupancy_grid
    # init occupancy grid
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "laser"
    #occupancy_grid.info.resolution = None # in m/cell
    occupancy_grid.info.resolution = 0.1 # in m/cell

    # width x height cells
    #occupancy_grid.info.width = None
    #occupancy_grid.info.height = None
    occupancy_grid.info.width = 900
    occupancy_grid.info.height = 900

    # origin is shifted at half of cell size * resolution
    occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
    occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
    occupancy_grid.info.origin.position.z = 0
    occupancy_grid.info.origin.orientation.x = 0
    occupancy_grid.info.origin.orientation.y = 0
    occupancy_grid.info.origin.orientation.z = 0
    occupancy_grid.info.origin.orientation.w = 1

    occupancy_grid.data = [0] * occupancy_grid.info.width * occupancy_grid.info.width


def scanCallback(scan_msg):
    global occupancy_grid

    # default_values()
    resetGrid()

    # convert scan measurements into an occupancy grid    
    #<>
    ranges = scan_msg.ranges
    angle = scan_msg.angle_min
    incr = scan_msg.angle_increment
    Xs, ys = [], []
    for r in ranges:
        constraint = angle < -0.5 and angle > -2.4
        if constraint and r != float("inf"):
            #r = r / occupancy_grid.info.resolution
            x = sin(angle) * r
            y = cos(angle) * r
            if True:#y < 0.2*x and y > -2*x and y < 0:
                setCell(x, y)
                Xs.append(x)
                ys.append(y)
        angle += incr

    if len(Xs) > 0 and len(ys) > 0:
        Xs = np.array(Xs).reshape(len(Xs), 1)
        ys = np.array(ys).reshape(len(ys), 1)

        model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
        try:
            #model_ransac.fit(Xs, ys)
            model_ransac.fit(ys, Xs)
            print("Coeffs: ", model_ransac.estimator_.coef_)
        except:
            print("NOT ENOUGH POINTS!")

        

#    for i in range(occupancy_grid.info.width):
#        setCell(i, 

    pub_grid.publish(occupancy_grid)

default_values()
rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=5)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=5)

# --- main ---
rospy.init_node("scan_grid")
rospy.spin()
