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

@dataclass(frozen=True)
class BYTETrackerArgs:
    track_thresh: float = 0.25
    track_buffer: int = 30
    match_thresh: float = 0.8
    aspect_ratio_thresh: float = 3.0
    min_box_area: float = 1.0
    mot20: bool = False

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

        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                    distortion_coefficients)

        cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

   
    return frame


    

aruco_type = "DICT_4X4_50"

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])

arucoParams = cv2.aruco.DetectorParameters_create()


intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
distortion = np.array((-0.43948,0.18514,0,0))


cap = cv2.VideoCapture(0)

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