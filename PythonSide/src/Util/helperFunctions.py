
from math import pi, atan2, sqrt, asin
import numpy as np
import json
import socket
from pyquaternion import Quaternion


# Sends data to unity
def sendData(rotation, centre, sock, ip, port):
    data = {
        "box": {
            "rotation": rotation.tolist(),
            "centre": centre.tolist(),
        },
    }

    packet = json.dumps(data, indent=3)
    sock.sendto( (packet).encode(), (ip, port) )

def radToDeg(rad):
    return rad * 180 / pi

def normaliseQuaternion(q):
    return q / np.linalg.norm(q)

# Multiply two quaternions (w,x,y,z) together 
def multiplyQuaternions(q0, q1):
    w0,x0,y0,z0 = q0
    w1,x1,y1,z1 = q1

    w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return normaliseQuaternion(np.array([w,x,y,z]))

# Convert a 3x3 rotation matrix to a quaternion (w,x,y,z)
def rotationMatrixToQuaternion(rotation):
    w = sqrt(1 + rotation[0,0] + rotation[1,1] + rotation[2,2]) / 2
    w4 = (4 * w)
    x = (rotation[2,1] - rotation[1,2]) / w4
    y = (rotation[0,2] - rotation[2,0]) / w4
    z = (rotation[1,0] - rotation[0,1]) / w4

    return normaliseQuaternion(np.array([w, x, y, z]))

def quaternionToEuler(rotation):
    w,x,y,z = rotation

    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = (-pi / 2.) + 2. * atan2 ( sqrt(1 + 2 * (w * y - x * z)), sqrt(1 - 2 * (w * y - x * z)))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2)) 

    return np.array([roll, pitch, yaw])

def quaternionInverse(q):
    new_q = Quaternion(q)
    return new_q.inverse
