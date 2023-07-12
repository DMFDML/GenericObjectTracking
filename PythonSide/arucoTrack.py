import sys
# sys.path.append("./sort_oh")
import numpy as np
import cv2
import time
from objectTrackingConstants import *
import socket

from math import atan2, cos, sin, sqrt, pi
from src.Camera.AzureKinect import AzureKinectCamera
from src.Camera.FakeCamera import FakeCamera
from src.Camera.OpenCVCamera import OpenCVCamera
import argparse
from ObjectTrackerInterface import ObjectTracker
import glob
from objectTrackingConstants import DISTANCE_CONVERSION
    


class ArucoTracker(ObjectTracker):

    def __init__(self, sock, camera, aruco_type, ip=UDP_IP, port=UDP_PORT, marker_size=40):
        self.camera = camera
        self.aruco_type = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
        
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.intrinsic, self.distortion = camera.get_calibration()


        self.sock = sock
        self.ip = ip
        self.port = port

        self.marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        self.marker_size = marker_size

        self.moving_average_rotation = [(0,0,0)]
        # self.moving_average_translation = [(0,0,0)]
        self.moving_average_length = 5
        self.moving_average_weights = np.array([0.2,0.2,0.4,0.4,0.6,0.6,0.8,0.8,1,1])
        self.display_values = []

        self.previous_rotation = np.array([0,0,0,0])
        self.previous_centre = np.array([0,0,0])

    def _moving_average(self, new_value):
        if len(self.moving_average_rotation) >= self.moving_average_length:
            self.moving_average_rotation.pop(0)

        self.moving_average_rotation.append(new_value)

        return np.mean(np.array(self.moving_average_rotation), axis=0)

    def _aruco_display(self, corners, ids, rvecs, tvecs,image, colour=(0, 0, 255)):
        
        if len(corners) > 0:
            
            ids = ids.flatten()
            
            for (markerCorner, markerID, rvec, tvec) in zip(corners, ids, rvecs, tvecs):
                
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
                cv2.drawFrameAxes(image, self.intrinsic, self.distortion, rvec, tvec, self.marker_size)  
                
        return image

    def _calculateRotationAndTranslation(self, corners, ids):
        rvecs, tvecs = [], []
        quaternions, centres = [], []
        
        for i in range(len(corners)):
            # Calculate the pose estimation for one of the detected markers
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.intrinsic,
                                                                    self.distortion)
            rvec, tvec = rvec.reshape(3), tvec.reshape(3)

            rvecs.append(rvec)
            tvecs.append(tvec)
            # Convert the rvec value to a 3x3 rotation matrix
            r = cv2.Rodrigues(rvec)
            # Convert the rotation matrix to Euler values (should probably just use Quaternion)
            quaternion = self._rotationMatrixToQuaternion(r[0])
            offcet = OFFCET_DICT.get(ids[i][0], {"translation": [0, 0, 0], "rotation": [0, 0, 0, 0]})


            # Add rotaiton and translation offcets to the angles depending on which aruco is being seen
            quaternion = self._multiplyQuaternions(offcet["rotation"], quaternion)
            centre = tvec + offcet['translation']

            quaternions.append(quaternion)
            centres.append(centre)

        return rvecs, tvecs, quaternions, np.asarray(centres)

    def _trackFrame(self, img):
        # Detect aruco markers
        (corners, ids, _) = cv2.aruco.detectMarkers(img, self.aruco_type, parameters=self.arucoParams)
        if len(corners) > 0:
            rvecs, tvecs, quaternions, centre = self._calculateRotationAndTranslation(corners, ids)

            # Calculate which aruco is closest to the camera
            closest_aruco = np.argmin(centre[:, 2])

            # moving_average_rvecs = self._moving_average(eulers[closest_aruco])
            self.display_values = [corners, ids, rvecs, tvecs]

            # Send data, / 1000 is due to opencv working in mm and unity working in m
            self.previous_rotation, self.previous_centre = (quaternions[closest_aruco], centre[closest_aruco] / DISTANCE_CONVERSION)
            return self.previous_rotation, self.previous_centre
        else:
            return self.previous_rotation, self.previous_centre
        


    def startTracking(self):
        started = False
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            try:
                img = self.camera.read()
            except:
                print("Can't read image from camera")
                break

            if started:
                rotation, centre = self._trackFrame(img)
                self.sendData(rotation, centre)

                self._aruco_display(self.display_values[0],self.display_values[1],self.display_values[2],self.display_values[3], img)

            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.flip(img, 1)
            cv2.imshow("tracking", img)

            # Press q to quit
            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
            if k & 0xFF == ord('b'):
                self.marker_size += 1
                self.marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                              [self.marker_size / 2, self.marker_size / 2, 0],
                              [self.marker_size / 2, -self.marker_size / 2, 0],
                              [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)
                print("Marker size: ", self.marker_size)
            if k == ord('s'):
                started = not started
        self.camera.stop()

    def benchMark(self):
        pass

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Aruco Object Tracking')
    parser.add_argument('--t', dest='type', default=ARUCO_TYPE,
                    help='The type of aruco marker being used')
    parser.add_argument('--ip', dest='ip', default=UDP_IP,
                    help='IP address of the computer to send data to')
    parser.add_argument('--port', dest='port', default=UDP_PORT,
                    help='Port of the computer to send data to')
    parser.add_argument('--isCamera', dest='isCamera', default=FAKE_CAMERA, type=bool,
                    help='If a camera is being used or not')
    parser.add_argument('--videoPath', dest='videoPath', default=None,
                    help='The path to the images being used')
    parser.add_argument('--cameraid', dest='cameraid', type=int, default=0, help="The camera ID")
    parser.add_argument('--calibration', dest='calibration', type=str, default='images\calibration\calibrate*.png', help="The path to images for camera calibration")
    parser.add_argument('--marker', dest='marker', type = int,
                        default=400, help='The size of the aruco marker being used')

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    if args.isCamera:
        if args.videoPath is None:
            camera = FakeCamera()
        else:
            camera = FakeCamera(videoPath=args.videoPath)
    else:
        # camera = AzureKinectCamera()
        camera = OpenCVCamera(camera_id=args.cameraid, calibration_files=args.calibration)

    
   
    objectTracker = ArucoTracker(sock, camera, args.type, ip=args.ip, port=5064, marker_size=args.marker)

    objectTracker.startTracking()
    
    cv2.destroyAllWindows()