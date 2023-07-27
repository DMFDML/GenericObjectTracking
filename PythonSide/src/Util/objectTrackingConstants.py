from cv2 import aruco
from math import pi, cos,sin, tan
from ObjectTrackers.ObjectTrackerInterface import ObjectTracker
# SiamMask Constants
CONFIG = './SiamMask/PreTrainedModels/config_VOT.json'
PRE_TRAINED_MODEL_SiamMask = './SiamMask/PreTrainedModels/SiamMask_VOT.pth'
from Util.helperFunctions import *

# Re3 Constants
PRE_TRAINED_MODEL_Re3 = './re3-tensorflow/logs/checkpoints'

# object tracking constants
TRACKER = "SiamMask"
VOXEL_SIZE = 0.04
COLOUR = False
NO_ITERATIONS = 30
FAKE_CAMERA = False
UDP_IP = "127.0.0.1"
UDP_PORT = 5065
AZURE_CALIBRATION = "./src/Camera/calibration_azure.json"
OPENCV_CALIBRATION = "./src/Camera/calibration_opencv.json"

# The conversion between camera space to world space, opencv and azure kinect use mm but unity uses m so have to convert between!!
DISTANCE_CONVERSION_AZURE_POINT_CLOUD = 100
DISTANCE_CONVERSION_AZURE_DEPTH = 1000
DISTANCE_CONVERSION_ARUCO = 1000

# Aruco Constants
ARUCO_TYPE = "DICT_4X4_50"
ARUCO_DICT = {
	"DICT_4X4_50": aruco.DICT_4X4_50,
	"DICT_4X4_100": aruco.DICT_4X4_100,
	"DICT_4X4_250": aruco.DICT_4X4_250,
	"DICT_4X4_1000": aruco.DICT_4X4_1000,
	"DICT_5X5_50": aruco.DICT_5X5_50,
	"DICT_5X5_100": aruco.DICT_5X5_100,
	"DICT_5X5_250": aruco.DICT_5X5_250,
	"DICT_5X5_1000": aruco.DICT_5X5_1000,
	"DICT_6X6_50": aruco.DICT_6X6_50,
	"DICT_6X6_100": aruco.DICT_6X6_100,
	"DICT_6X6_250": aruco.DICT_6X6_250,
	"DICT_6X6_1000": aruco.DICT_6X6_1000,
	"DICT_7X7_50": aruco.DICT_7X7_50,
	"DICT_7X7_100": aruco.DICT_7X7_100,
	"DICT_7X7_250": aruco.DICT_7X7_250,
	"DICT_7X7_1000": aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": aruco.DICT_APRILTAG_36h11
}
OFFCET_DICT = {
    7: { # front, Piston 1
        "translation": [0, 0, 0],
        "rotation": [cos(0), 0, sin(0), sin(0)]
    },
    11: { # back, Piston 2
        "translation": [0, 0, 0],
        "rotation": [cos(0.5 * pi), 0, sin(0.5 * pi), 0]
    },
    13: { # left, Piston 3
        "translation": [0, 0, 0],
        "rotation": [cos(0.5 * -pi/2), 0, sin(0.5 * -pi/2), 0]
    },
    6: { # right, Link 1 End
        "translation": [0, 0, 0],
        "rotation": [cos(0.5 * pi/2), 0, sin(0.5 * pi/2), 0]
    },
    10: { # top, Link 2 End
        "translation": [0, 0, 0],
        "rotation": [cos(0.5 * -pi /2), sin(0.5 * -pi/2), 0, 0]
    },
    12: { # bottom, Link 3 End
        "translation": [0, 0, 0],
        "rotation": [cos(0.5 * pi /2), sin(0.5 * pi/2), 0,0]
    },

}
ARUCO_ROTATION_OFFCET = multiplyQuaternions([cos(0.5 * pi), 0, sin(0.5 * pi), 0], [cos(0.5 * pi), 0, 0, sin(0.5 * pi)])
# ARUCO_ROTATION_OFFCET = [1,0,0,0]

CHECKERBOARD = (6,9)
SMALL_ANGLE_VALUE = 1

# Fake Camera Constants
IMAGE_FILE_LOCATIONS = "./images/benchmark_images/"

# Benchmark Constants
BENCHMARK_FILE = "./benchmark/benchmark_result"


