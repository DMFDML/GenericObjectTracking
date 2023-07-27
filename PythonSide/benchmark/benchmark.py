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

from math import pi, cos,sin, atan2, acos

LAZY_SUSAN_PIXEL_RADIUS = 200
LAZY_SUSAN_RADIUS = 0.19
CAMERA_HEIGHT = 0.48
# LAZY_SUSAN_CENTRE = (640//2, 560//2)
LAZY_SUSAN_CENTRE = (1280//2, 720//2)
LAZY_SUSAN_ROTATION = 1/17
# The offcet for the lazy susan rotation (degree the lazy susan starts)
FRAME_OFFSET = 0
# The starting image for the fake camera 
INITIAL_FRAME = 0
# The starting image for the rotation, but not the camera !!!!! (value 8 is used for vive tracker)
ROTATION_START_IMAGE = 8


def pickTracker(args):
    if args.tracker == "GenericObjectTracker":
        camera = FakeCamera(transformed=True, imageFilePath=args.videoPath, voxel_size=args.voxel_size, initial_frame=INITIAL_FRAME)

        # parse the box argument into a list of ints (from '[x,y,w,h]' to [x,y,w,h]])
        box = [int(x) for x in args.box]
        if box == []:
            img = camera.read()
            # camera.currentImage = 0
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
        
        return camera, ReflectorTrack(None, camera, voxel_size=0.02, no_iterations=30, colour=True)

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

def lazySusanRotation(file, img, frame, rotation, translation, end, start):
    # Rotation = 2 * pi * rotation speed * frame rate * current frame
    lazy_rotation = 2 * pi * LAZY_SUSAN_ROTATION * 1 / 30 * (frame +FRAME_OFFSET)
    x,y, z = LAZY_SUSAN_RADIUS * cos(lazy_rotation), LAZY_SUSAN_RADIUS * sin(lazy_rotation), CAMERA_HEIGHT
    pixel_x,pixel_y = LAZY_SUSAN_PIXEL_RADIUS * cos(lazy_rotation), LAZY_SUSAN_PIXEL_RADIUS * sin(lazy_rotation)

    roll,pitch,yaw = quaternionToEuler(rotation) 


    translation_error = np.sqrt((x - translation[0])**2 + (y - translation[1])**2)
    rotation_error_general = np.sqrt((acos(cos(roll)) - 0) ** 2 + (acos(cos(pitch)) - 0) ** 2 + (acos(cos(yaw)) - acos(cos(lazy_rotation))) ** 2)
    rotation_error_z = np.sqrt((acos(cos(yaw)) - acos(cos(lazy_rotation))) ** 2)

    print(radToDeg(lazy_rotation), " ",radToDeg(yaw), " ", radToDeg(acos(cos(lazy_rotation))), " ", radToDeg(acos(cos(yaw))))
    print(translation, " ", x, y, z)
    # rotation_error = np.sqrt((lazy_rotation - rotation)**2)
    file.write(f"{frame},{end - start},{rotation_error_general},{rotation_error_z},{translation_error}\n")

    
    cv2.line(img, (LAZY_SUSAN_CENTRE[0]+ int(pixel_x), LAZY_SUSAN_CENTRE[1] +int(pixel_y)), (LAZY_SUSAN_CENTRE[0]+int(pixel_x + 40 *cos(roll)) , LAZY_SUSAN_CENTRE[1]+int(pixel_y + 40 *sin(roll))), 255, 2)
    cv2.line(img, (LAZY_SUSAN_CENTRE[0]+ int(pixel_x), LAZY_SUSAN_CENTRE[1] +int(pixel_y)), (LAZY_SUSAN_CENTRE[0]+int(pixel_x + 40 *cos(pitch)) , LAZY_SUSAN_CENTRE[1]+int(pixel_y + 40 *sin(pitch))), 255, 2)
    cv2.line(img, (LAZY_SUSAN_CENTRE[0]+ int(pixel_x), LAZY_SUSAN_CENTRE[1] +int(pixel_y)), (LAZY_SUSAN_CENTRE[0]+int(pixel_x + 40 *cos(yaw)) , LAZY_SUSAN_CENTRE[1]+int(pixel_y + 40 *sin(yaw))), 255, 2)
    # Showing the rotation of the lazy susan
    cv2.line(img, LAZY_SUSAN_CENTRE, (LAZY_SUSAN_CENTRE[0]+int(pixel_x), LAZY_SUSAN_CENTRE[1]+int(pixel_y)), 255, 2)
    cv2.putText(img, f"Rotation: {lazy_rotation}", LAZY_SUSAN_CENTRE, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2)

    return img

def viveRotation(file, vive_file, initial_translation, frame, rotation, translation, end, start):
    line = vive_file.readline().split(",")
    vive_t = initial_translation - np.fromstring(line[2][1:-1], sep=' ')
    vive_r = np.fromstring(line[3][1:-1], sep=' ')

    roll,pitch,yaw = quaternionToEuler(rotation) 
    vive_roll,vive_pitch,vive_yaw = quaternionToEuler(vive_r) 


    translation_error = np.sqrt((vive_t[0] - translation[0])**2 + (vive_t[1] - translation[1])**2 + (vive_t[2] - translation[2])**2)
    rotation_error_general = np.sqrt((acos(cos(roll)) - acos(cos(vive_roll))) ** 2 + (acos(cos(pitch)) - acos(cos(vive_pitch))) ** 2 + (acos(cos(yaw)) - acos(cos(vive_yaw))) ** 2)
    rotation_error_z = np.sqrt((acos(cos(yaw)) - acos(cos(vive_yaw))) ** 2)

    print(radToDeg(quaternionToEuler(rotation) ), " ", radToDeg(quaternionToEuler(vive_r)))
    print(translation, " ", vive_t)
    # rotation_error = np.sqrt((lazy_rotation - rotation)**2)
    file.write(f"{frame},{end - start},{rotation_error_general},{rotation_error_z},{translation_error}\n")


def benchMark(file, tracker_method, camera, vive_file):
    # benchmark the tracker
    # Input: speed_file, The open file for frame rate data
    #        translation_rotation, The open file for tranlation and rotation data
    #        tracker_method, The tracker being used
    #        camera, The camera being used
    # Output: None

    file.write("Frame,Time,Rotation,Rotation_z,Translation\n" )
    frame = 0
    initial_translation = np.array([0.,0.,0.])
    camera.currentImage = ROTATION_START_IMAGE

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
            img, depth = img
            start = time.time()
            rotation, translation = tracker_method.trackFrame(img,depth)
        
        end = time.time()
        

        if vive_file is None:
            if len(img) != 2:
                img = lazySusanRotation(file, img, frame, rotation, translation, end, start)
            else:
                img = lazySusanRotation(file, img[0], frame, rotation, translation, end, start)
        else:

            if np.all(initial_translation == np.array([0.,0.,0.])):
                initial_translation = translation
                print("Initial Transformation", initial_translation)

            viveRotation(file, vive_file, initial_translation, frame, rotation, translation, end, start)

        frame += 1        

        cv2.imshow("tracking", img)
        k = cv2.waitKey( 1)
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

    file = open(args.videoPath + "../" + f"{objectTracker.name}.csv", "a")
    print(file)
    camera.currentImage = 0
    benchMark(file, objectTracker, camera, vive_file)

    file.close()
    if args.vive:
        vive_file.close()

