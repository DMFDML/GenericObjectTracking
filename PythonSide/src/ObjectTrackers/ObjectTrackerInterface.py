from abc import ABC, abstractmethod
import json
import socket
from math import pi, atan2, sqrt
import numpy as np

class ObjectTracker(ABC):
    
    # Sends data to unity
    def sendData(self, rotation, centre):
        data = {
            "box": {
                "rotation": rotation.tolist(),
                "centre": centre.tolist(),
            },
        }

        packet = json.dumps(data, indent=3)
        self.sock.sendto( (packet).encode(), (self.ip, self.port) )

    def _radToDeg(self, rad):
        return rad * 180 / pi
    
    # Multiply two quaternions (w,x,y,z) together 
    def _multiplyQuaternions(self, q0, q1):
        w0,x0,y0,z0 = q0
        w1,x1,y1,z1 = q1

        w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        return np.array([w,x,y,z])
    
    # Convert a 3x3 rotation matrix to a quaternion (w,x,y,z)
    def _rotationMatrixToQuaternion(self, rotation):
        w = sqrt(1 + rotation[0,0] + rotation[1,1] + rotation[2,2]) / 2
        w4 = (4 * w)
        x = (rotation[2,1] - rotation[1,2]) / w4
        y = (rotation[0,2] - rotation[2,0]) / w4
        z = (rotation[1,0] - rotation[0,1]) / w4

        return np.array([w, x, y, z])

    @abstractmethod
    def trackFrame(self, img):
        # start tracking the object
        # Input: img, The frame of the video being processed
        # Output: rotation, The rotation of the object
        #         translation, The translation of the object
        pass

    @abstractmethod
    def startTracking(self):
        # start tracking the object
        # Input: None
        # Output: None
        pass

