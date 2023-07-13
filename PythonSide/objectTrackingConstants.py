from cv2 import aruco
from math import pi
# SiamMask Constants
CONFIG = './SiamMask/PreTrainedModels/config_VOT.json'
PRE_TRAINED_MODEL_SiamMask = './SiamMask/PreTrainedModels/SiamMask_VOT.pth'

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

# The conversion between camera space to world space, opencv and azure kinect use mm but unity uses m so have to convert between!!
DISTANCE_CONVERSION = 100

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
    0: { # front
        "translation": [0, 0, 0],
        "rotation": [0, 0, 0, 0]
    },
    1: { # back
        "translation": [0, 0, 0],
        "rotation": [0, 0, 1, 0]
    },
    2: { # left
        "translation": [0, 0, 0],
        "rotation": [0, 1, 0, 0]
    },
    3: { # right
        "translation": [0, 0, 0],
        "rotation": [0, -1, 0, 0]
    },
    4: { # top
        "translation": [0, 0, 0],
        "rotation": [0, 0, 0, 1]
    },
    6: { # bottom
        "translation": [0, 0, 0],
        "rotation": [0, 0, 0, 1]
    },
    11: { # anotherOne
        "translation": [0, 0, 0],
        "rotation": [0, 0, 1, 0]
    }

}

CHECKERBOARD = (6,9)

# Fake Camera Constants
IMAGE_FILE_LOCATIONS = "../TestCode/images/"

# Benchmark Constants
TIME_FILE = "./benchmark/time.csv"
TRANSLATION_FILE = "./benchmark/translation_rotation.csv"
