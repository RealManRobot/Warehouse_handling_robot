import numpy as np
'''
基于原坐标系的x,y,z旋转, 不是分步骤旋转.
'''

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions.
    q1 = (x1, y1, z1, w1)
    q2 = (x2, y2, z2, w2)
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = x1*w2 + y1*z2 - z1*y2 + w1*x2
    y = -x1*z2 + y1*w2 + z1*x2 + w1*y2
    z = x1*y2 - y1*x2 + z1*w2 + w1*z2
    w = -x1*x2 - y1*y2 - z1*z2 + w1*w2
    return np.array([x, y, z, w])

def euler_to_quaternion(rx, ry, rz):
    """
    Convert Euler angles (in degrees) to quaternion.
    """
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    crx, cry, crz = np.cos([rx/2, ry/2, rz/2])
    srx, sry, srz = np.sin([rx/2, ry/2, rz/2])
    x = srx * cry * crz - crx * sry * srz
    y = crx * sry * crz + srx * cry * srz
    z = crx * cry * srz - srx * sry * crz
    w = crx * cry * crz + srx * sry * srz
    return np.array([x, y, z, w])

w = 0.548521
x = -0.071119
y = 0.434545
z = -0.710801

# Given initial quaternion of A with respect to B
q1 = np.array([x, y, z, w])  # Replace x, y, z, w with actual values

# Additional rotation in degrees
rx = -90
ry = 0
rz = -90

# Convert Euler angles to quaternion
q2 = euler_to_quaternion(rx, ry, rz)

# Multiply the quaternions
q_final = quaternion_multiply(q1, q2)

print("Final quaternion of A with respect to B:", q_final)


string = "camera_right"
print(string.split("_")[-1])

print([0.0] * 3)