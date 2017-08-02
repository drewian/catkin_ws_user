#!/usr/bin/env python

# --- imports ---
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from math import sin, cos

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
    #occupancy_grid.header.frame_id = "map" #"laser"
    #occupancy_grid.info.resolution = None # in m/cell
    occupancy_grid.info.resolution = 0.35 # in m/cell

    # width x height cells
    #occupancy_grid.info.width = None
    #occupancy_grid.info.height = None
    occupancy_grid.info.width = 600
    occupancy_grid.info.height = 600

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
    ranges = scan_msg.ranges
    angle = scan_msg.angle_min
    incr = scan_msg.angle_increment
    for r in ranges:
        if r != float("inf"):
            r = r / occupancy_grid.info.resolution
            x = sin(angle) * r
            y = cos(angle) * r
            setCell(x, y)
        angle += incr

    pub_grid.publish(occupancy_grid)

default_values()
rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=5)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=5)

# --- main ---
rospy.init_node("scan_grid")
rospy.spin()
