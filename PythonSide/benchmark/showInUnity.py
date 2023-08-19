import sys
sys.path.append('./src/')
from ObjectTrackers.genericObjectTracker import GenericObjectTracker
from ObjectTrackers.reflectorTrack import ReflectorTrack
from ObjectTrackers.reflectorTrackNoICP import ReflectorTrackNoICP
from ObjectTrackers.arucoTrack import ArucoTracker
from Trackers2D.SiamMask import SiamMaskSharpTracker
from Trackers2D.Re3 import Re3Tracker
from Camera.AzureKinect import AzureKinectCamera
from Camera.FakeCamera import FakeCamera
import argparse
import time

import socket

from Util.objectTrackingConstants import *
import numpy as np
import cv2

from math import pi, cos,sin, atan2, acos


FRAME_OFFSET = 0
INITIAL_FRAME = 0
ROTATION_START_IMAGE = 8

FRAME_RATE = 1 / 10



def pickTracker(args):
    if args.tracker == "GenericObjectTracker":
        camera = FakeCamera(transformed=True, imageFilePath=args.videoPath, voxel_size=args.voxel_size, initial_frame=INITIAL_FRAME)

        # parse the box argument into a list of ints (from '[x,y,w,h]' to [x,y,w,h]])
        box = [int(x) for x in args.box]
        if box == []:
            img = camera.read()
            box = cv2.selectROI("tracking", img, False)
            print(box)

        if TRACKER == "SiamMask":
            tracker = SiamMaskSharpTracker()
        elif TRACKER == "Re3":
            tracker = Re3Tracker()
        else:
            raise ValueError("Invalid tracker type")

        return camera, GenericObjectTracker(None, tracker, camera, box = box, colour= args.colour, no_iterations=args.no_iterations, voxel_size=args.voxel_size)
        
    elif args.tracker == "ReflectorTracker":
        camera = FakeCamera(transformed=False, imageFilePath=args.videoPath, voxel_size=0.02, min_standard_deviation=1, point_cloud_threshold=1000, initial_frame=INITIAL_FRAME)
        
        return camera, ReflectorTrack(None, camera, voxel_size=0.02, no_iterations=30, colour=False)

    elif args.tracker == "ReflectorTrackerNoICP":
        camera = FakeCamera(transformed = False, imageFilePath=args.videoPath, initial_frame=INITIAL_FRAME)
        return camera, ReflectorTrackNoICP(None, camera)

    elif args.tracker == "ArucoTracker":
        camera = FakeCamera(transformed=True, imageFilePath=args.videoPath, voxel_size=args.voxel_size, initial_frame=INITIAL_FRAME)

        return  camera, ArucoTracker(None, camera, args.aruco_type, marker_size=args.marker_shape) 
    else:
        raise ValueError("Invalid tracker type")

def getImage(tracker_type, camera):
    # Get the image from the camera
    # Input: tracker_type, The type of tracker being used
    # Output: img, The image from the camera

    if tracker_type == "GenericObjectTracker":
        return camera.read().copy()
    elif tracker_type == "ReflectorTracker":
        camera.read().copy()
        ir = camera.getIRimage().copy()

        # Threshold the image so that only the reflectors are visible
        return np.asarray(ir[:, :] > 20000, dtype=np.uint8) * 255
    elif tracker_type == "ReflectorTrackerNoICP":
        camera.read().copy()
        ir = camera.getIRimage().copy()
        depth = camera.getDepthimage().copy()

        # Threshold the image so that only the reflectors are visible
        ir_thresholded = np.asarray(ir[:, :] > 20000, dtype=np.uint8) * 255
        return ir_thresholded, depth
    elif tracker_type == "ArucoTracker":
        return camera.read().copy()        

def sendData(rotation, translation, sock, ip = "127.0.0.1", port = 5065):
    data = {
        "box": {
            "rotation": rotation.tolist(),
            "centre": translation.tolist(),
        },
    }

    packet = json.dumps(data, indent=3)
    sock.sendto( (packet).encode(), (ip, port) )

def benchMark(tracker_method, camera, vive_file):
    # benchmark the tracker
    # Input: translation_rotation, The open file for tranlation and rotation data
    #        tracker_method, The tracker being used
    #        camera, The camera being used
    # Output: None
    frame = FRAME_OFFSET
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera.currentImage = ROTATION_START_IMAGE
    initial_transformation = np.array([0,0,0])

    while True:
        # Start timer for fps calculation
        start = time.time()
        try:
            img = getImage(tracker_method.name, camera)
            
        except:
            print("Out of Images")
            break
        
        if len(img) != 2:
            rotation, translation = tracker_method.trackFrame(img)
        else:
            img, depth = img
            rotation, translation = tracker_method.trackFrame(img,depth)
        

        if vive_file is None:
            sendData(rotation, translation, sock, "127.0.0.1", 5065)
            
        else:
            line = vive_file.readline().split(",")
            vive_t = np.fromstring(line[2][1:-1], sep=' ')
            vive_r = np.fromstring(line[3][1:-1], sep=' ')

            if np.all(initial_transformation == vive_t):
                initial_transformation = translation
                print("Initial Transformation", initial_transformation)

            vive_t = initial_transformation - vive_t
            print(vive_t, translation)
            print(vive_r, rotation)
            sendData(rotation, translation, sock, "127.0.0.1", 5065)
            sendData(vive_r, vive_t, sock, "127.0.0.1", 5064)

        speed = (time.time() - start)
        time.sleep(max(0, FRAME_RATE - speed))
        frame += 1        

        cv2.imshow("tracking", img)
        k = cv2.waitKey( 0)
        if k & 0xFF == ord('q'):
            break

        print(f"Computing Frame {frame}")
        

    camera.stop()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark Values.')
    # parser.add_argument('--box', dest='box', default=[789, 342, 96, 73], type=list, 
    parser.add_argument('--box', dest='box', default=[], type=list, 
                    help='The location of the bounding box')
    parser.add_argument('--tracker', dest='tracker', default='GenericObjectTracker',
                    help='Type of tracker being used')
    parser.add_argument('--file', dest='videoPath', default=IMAGE_FILE_LOCATIONS,
                    help='The path to the images being used')
    parser.add_argument('--voxel_size', dest='voxel_size', default=VOXEL_SIZE,
                    help='The size of the voxels being used')
    parser.add_argument('--colour', dest='colour', type=bool, default=COLOUR,
                    help='If colour ICP is being used or not')
    parser.add_argument('--no_iterations', dest='no_iterations', default=NO_ITERATIONS,
                    help='The number of iterations for ICP')
    parser.add_argument('--aruco_type', dest='aruco_type', default=ARUCO_TYPE,
                    help='The type of the arucos being used')
    parser.add_argument('--marker_shape', dest='marker_shape', default=40,
                    help='The size of the arucos being used')
    parser.add_argument('--vive', dest='vive', default=False, type=bool,
                    help='If vive tracker is being used or not')
    args = parser.parse_args()
    
    vive_file = None
    if args.vive:
        vive_file = open(args.videoPath + "vive.csv", "r")
    

    camera, objectTracker = pickTracker(args)

    benchMark(objectTracker, camera, vive_file)

    if args.vive:
        vive_file.close()

