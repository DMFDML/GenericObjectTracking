import sys
sys.path.append("./src")
sys.path.append(".")
sys.path.append("./triad_openvr")
from triad_openvr import *
import time
import sys
import json
import socket
import numpy as np
from Util.helperFunctions import *
from math import cos, sin, pi

v = triad_openvr()
v.print_discovered_objects()

def sendData(rotation, centre, sock, ip = "127.0.0.1", port = 5064):
    data = {
        "box": {
            "rotation": rotation.tolist(),
            "centre": centre.tolist(),
        },
    }

    packet = json.dumps(data, indent=3)
    sock.sendto( (packet).encode(), (ip, port) )


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if len(sys.argv) == 1:
    interval = 1/250
elif len(sys.argv) == 2:
    interval = 1/float(sys.argv[1])
else:
    print("Invalid number of arguments")
    interval = False
    
if interval:

    vive = v.devices["tracker_1"].get_pose_quaternion()

    initial_translation = np.array(vive[:3])
    initial_rotation = np.array([vive[3], vive[4], vive[5], vive[6]])

    # initial_rotation = multiplyQuaternions(np.array([cos(0.5* (pi/2)), sin(0.5 *(pi/2)), 0, 0]), quaternionInverse(np.array(vive[3:])))
    # initial_rotation = multiplyQuaternions(np.array([cos(0.5* (pi/2)), 0, sin(0.5 *(pi/2)), 0]), quaternionInverse(np.array(vive[3:])))

    while(True):
        start = time.time()
        vive = v.devices["tracker_1"].get_pose_quaternion()
        translation = np.array(vive[:3]) - initial_translation
        rotation = multiplyQuaternions(initial_rotation, quaternionInverse(np.array([vive[3], vive[4], vive[5], vive[6]])) )
        rotation[1] = -rotation[1]
        # rotation = np.array([vive[3], vive[4], vive[6], vive[5]])

        print(translation, quaternionToEuler(rotation))
        sendData(rotation, translation, sock)
        
        sleep_time = interval-(time.time()-start)
        if sleep_time>0:
            time.sleep(sleep_time)