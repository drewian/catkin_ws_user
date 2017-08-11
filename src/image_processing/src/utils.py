import numpy as np
from math import cos, sin


def get2DRotationMatrix(angle):
    return np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])