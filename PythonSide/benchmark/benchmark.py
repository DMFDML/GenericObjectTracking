import sys
sys.path.append('./')
from genericObjectTracker import GenericObjectTracker
from SiamMask import SiamMaskSharpTracker
from Re3 import Re3Tracker
from src.Camera.AzureKinect import AzureKinectCamera
from src.Camera.FakeCamera import FakeCamera
import argparse

from objectTrackingConstants import *

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Benchmark Values.')
    parser.add_argument('--box', dest='box', default=[], required=True,
                    help='The location of the bounding box')
    parser.add_argument('--tracker', dest='tracker', default='SiamMask',
                    help='Type of tracker being used')
    parser.add_argument('--videoPath', dest='videoPath', default=IMAGE_FILE_LOCATIONS,
                    help='The path to the images being used')
    parser.add_argument('--voxel_size', dest='voxel_size', default=VOXEL_SIZE,
                    help='The size of the voxels being used')
    parser.add_argument('--colour', dest='colour', type=bool, default=COLOUR,
                    help='If colour ICP is being used or not')
    parser.add_argument('--no_iterations', dest='no_iterations', default=NO_ITERATIONS,
                    help='The number of iterations for ICP')
    


    args = parser.parse_args()
    # parse the box argument into a list of ints (from '[x,y,w,h]' to [x,y,w,h]])
    box = [int(x) for x in args.box.split('[')[1].split(']')[0].split(',')]
    print(box)

    if args.tracker == "SiamMask":
        tracker = SiamMaskSharpTracker()
    elif args.tracker == "Re3":
        tracker = Re3Tracker()
    else:
        raise ValueError("Invalid tracker type")

    camera = FakeCamera(imageFilePath=args.videoPath, voxel_size=args.voxel_size)
    
    objectTracker = GenericObjectTracker(None, tracker, camera, box = box, colour= args.colour, no_iterations=args.no_iterations, voxel_size=args.voxel_size)

    speed_file = open(TIME_FILE, "a")
    translation_rotation = open(TRANSLATION_FILE, "a")

    objectTracker.benchMark(speed_file, translation_rotation)



    speed_file.close()
    translation_rotation.close()
