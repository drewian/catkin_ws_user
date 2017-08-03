from math import atan, acos, sin, pi, sqrt
import sys

d2 = float(sys.argv[1])
m = float(sys.argv[2])


angle = atan(m) - atan(-0.02)

R = (d2 - 1) / sin(angle)

alpha = pi - acos(1/R)
gamma = atan((d2 + 1) / sqrt(R ** 2 + (d2 + 1) ** 2 + (d2 + 1) * R * sin(alpha)))

print(R, alpha, gamma)
