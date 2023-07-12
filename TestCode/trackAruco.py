import sys
# sys.path.append("./sort_oh")
import numpy as np
import cv2
import sys
import time
# from bytetracker import BYTETracker
from dataclasses import dataclass
# from sort.tracker import SortTracker
# from deep_sort_realtime.deepsort_tracker import DeepSort
# from sort_oh.tracker import Sort_OH
import glob

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

CHECKERBOARD = (6, 9)
def cameraCalibration():
    
    # stop the iteration when specified
    # accuracy, epsilon, is reached or
    # specified number of iterations are completed.
    criteria = (cv2.TERM_CRITERIA_EPS + 
                cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    
    # Vector for 3D points
    threedpoints = []
    
    # Vector for 2D points
    twodpoints = []
    
    
    #  3D points real world coordinates
    objectp3d = np.zeros((1, CHECKERBOARD[0] 
                        * CHECKERBOARD[1], 
                        3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    
    
    # Extracting path of individual image stored
    # in a given directory. Since no path is
    # specified, it will take current directory
    # jpg files alone
    images = glob.glob('images/calibration/calibrate*.png')
    
    for filename in images:
        image = cv2.imread(filename)
        grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        # If desired number of corners are
        # found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(
                        grayColor, CHECKERBOARD, 
                        cv2.CALIB_CB_ADAPTIVE_THRESH 
                        + cv2.CALIB_CB_FAST_CHECK + 
                        cv2.CALIB_CB_NORMALIZE_IMAGE)
    
        # If desired number of corners can be detected then,
        # refine the pixel coordinates and display
        # them on the images of checker board
        if ret == True:
            threedpoints.append(objectp3d)
    
            # Refining pixel coordinates
            # for given 2d points.
            corners2 = cv2.cornerSubPix(
                grayColor, corners, (11, 11), (-1, -1), criteria)
    
            twodpoints.append(corners2)
    
    # Perform camera calibration by
    # passing the value of above found out 3D points (threedpoints)
    # and its corresponding pixel coordinates of the
    # detected corners (twodpoints)
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
        threedpoints, twodpoints, grayColor.shape[::-1], None, None)
    
    
    # Displaying required output
    print(" Camera matrix:")
    print(matrix)
    
    print("\n Distortion coefficient:")
    print(distortion)
    return matrix, distortion


def draw_bounding_boxs(img, boxes, colour=(0, 255, 255)):
    for b in boxes:
        x1,y1,x2,y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
        cv2.rectangle(img, (x1,y1), (x2, y2), colour, 3, 1)

def aruco_display(corners, ids, rejected, image, colour=(0, 0, 255)):
    
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, colour, 2)
			cv2.line(image, topRight, bottomRight, colour, 2)
			cv2.line(image, bottomRight, bottomLeft, colour, 2)
			cv2.line(image, bottomLeft, topLeft, colour, 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			# print("[Inference] ArUco marker ID: {}".format(markerID))
			
	return image


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
    
    # cv2.aruco.drawDetectedMarkers(frame, corners) 
    
    max_points = np.array([[corners[i][0].min(axis = 0)[0], 
                    corners[i][0].min(axis = 0)[1],
                    corners[i][0].max(axis = 0)[0], 
                    corners[i][0].max(axis = 0)[1], 0.98] for i in range(len(corners))])
    # print("Max_points", max_points)
    # bounding_polygon = corners[:]
    # bounding_polygon = np.asarray(corners).reshape((-1,8))
    # bounding_polygon = [[bounding_polygon[i], 0.98] for i in range(len(bounding_polygon))]
    aruco_display(corners, ids, rejected_img_points, frame)
    draw_bounding_boxs(frame, max_points)
    
    # if len(max_points) > 0:

    #     tracks, unmatched = tracker.update(max_points, frame.shape[:2])
    #     print("Tracks", tracks, "Unmatched", unmatched)
    #     draw_bounding_boxs(frame, tracks, (255, 255, 0))
    #     draw_bounding_boxs(frame, tracks, (0, 255, 0))

        
    for i in range(0, len(corners)):

        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 400, matrix_coefficients,
                                                                    distortion_coefficients)

        cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 400)  
        print(rvec)
   
    return frame


    

aruco_type = "DICT_4X4_250"

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters_create()


intrinsic_camera, distortion = cameraCalibration()

cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# tracker = Sort_OH(max_age=30)



while cap.isOpened():
    timer = cv2.getTickCount()
    ret, img = cap.read()
    
    output = pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)

    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    cv2.putText(output, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow('Estimated Pose', output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()