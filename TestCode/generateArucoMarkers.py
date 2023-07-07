import cv2
from cv2 import aruco
import argparse
import numpy as np

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

ARUCO_PATH = "./images/aruco"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--output", default=ARUCO_PATH, type=str,
        help="path to output image containing ArUCo tag")
    ap.add_argument("-i", "--id", type=int, default = 10,
        help="Number of Aruco tag to generate")
    ap.add_argument("-t", "--type", type=str,
        default="DICT_4X4_50",
        help="type of ArUCo tag to generate")
    ap.add_argument("-s", "--size", type=int, default=100,
        help="The size of the generated aruco markers")
    args = vars(ap.parse_args())

    arucoDict = aruco.Dictionary_get(ARUCO_DICT[args["type"]])

    for size in [75, 100, 250]:
        for i in range(args.get("id")):
            tag = np.zeros((size,size, 1), dtype="uint8")
            aruco.drawMarker(arucoDict, i, args.get("size"), tag, 1)
            cv2.imwrite(f"{args.get('output')}/{size}aruco{i}.png", tag)



