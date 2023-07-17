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

from Util.objectTrackingConstants import *
import numpy as np
import cv2

def pickTracker(args):
    if args.tracker == "GenericObjectTracker":
        camera = FakeCamera(transformed=True, imageFilePath=args.videoPath, voxel_size=args.voxel_size)
        # parse the box argument into a list of ints (from '[x,y,w,h]' to [x,y,w,h]])
        box = [int(x) for x in args.box]
        if box == []:
            img = camera.read()
            camera.currentImage = 0
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
        camera = FakeCamera(transformed=False, imageFilePath=args.videoPath, voxel_size=args.voxel_size, min_standard_deviation=1, point_cloud_threshold=2000)
        
        return camera, ReflectorTrack(None, camera, voxel_size=args.voxel_size, no_iterations=args.no_iterations, colour=False)

    elif args.tracker == "ReflectorTrackerNoICP":
        camera = FakeCamera(transformed = False)
        return camera, ReflectorTrackNoICP(None, camera)

    elif args.tracker == "ArucoTracker":
        camera = FakeCamera(transformed=True, imageFilePath=args.videoPath, voxel_size=args.voxel_size)

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

def benchMark(file, tracker_method, camera):
    # benchmark the tracker
    # Input: speed_file, The open file for frame rate data
    #        translation_rotation, The open file for tranlation and rotation data
    #        tracker_method, The tracker being used
    #        camera, The camera being used
    # Output: None

    file.write("Frame, Time, Rotation, Translation\n" )
    frame = 0
    while True:
        # Start timer for fps calculation
        try:
            img = getImage(tracker_method.name, camera)
            
        except:
            print("Out of Images")
            break
        
        if len(img) != 2:
            start = time.time()
            rotation, translation = tracker_method.trackFrame(img)
        else:
            start = time.time()
            rotation, translation = tracker_method.trackFrame(img[0], img[1])
        
        end = time.time()

        file.write(f"{frame}, {end - start}, {rotation}, {translation}, \n")
        frame += 1
        

        print(f"Computing Frame {frame}")
        

    camera.stop()
    file.write("\n\n")

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark Values.')
    parser.add_argument('--box', dest='box', default=[], type=list, 
                    help='The location of the bounding box')
    parser.add_argument('--tracker', dest='tracker', default='GenericObjectTracker',
                    help='Type of tracker being used')
    parser.add_argument('--videoPath', dest='videoPath', default=IMAGE_FILE_LOCATIONS,
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
    args = parser.parse_args()
    
    

    

    camera, objectTracker = pickTracker(args)

    file = open(BENCHMARK_FILE + f"_{objectTracker.name}.csv", "a")
    camera.currentImage = 0
    benchMark(file, objectTracker, camera)

    file.close()

