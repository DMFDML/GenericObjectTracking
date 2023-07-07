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
import argparse
from ObjectTrackerInterface import ObjectTracker



class ArucoTracker(ObjectTracker):

    def __init__(self, sock, camera, aruco_type, ip=UDP_IP, port=UDP_PORT, marker_size=1):
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
        self.moving_average_length = 10
        self.moving_average_weights = np.array([0.2,0.2,0.4,0.4,0.6,0.6,0.8,0.8,1,1])

        
    def _moving_average(self, new_value):
        if len(self.moving_average_rotation) >= self.moving_average_length:
            self.moving_average_rotation.pop(0)
        if new_value[0] < 0:
            new_value[0] += 2*pi
        if new_value[1] < 0:
            new_value[1] += 2*pi
        if new_value[2] < 0:
            new_value[2] += 2*pi


        self.moving_average_rotation.append(new_value)

        return np.sum(np.asarray(self.moving_average_rotation) * self.moving_average_weights.reshape(-1, 1)[:len(self.moving_average_rotation)], axis=0) / np.sum(self.moving_average_weights)

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

    def _radToDeg(self, rad):
        return rad * 180 / pi

    def _calculateRotationAndTranslation(self, corners, ids):
        rvecs, tvecs = [], []
        centre = []
        
        for i in range(len(corners)):

            _, rvec, tvec = cv2.solvePnP(self.marker_points, corners[i], self.intrinsic,self.distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            offcet = OFFCET_DICT.get(ids[i][0], {"translation": [0, 0, 0], "rotation": [0, 0, 0]})
            rvec, tvec = rvec.reshape((1, 3)), tvec.reshape((1, 3))

            rvec[0] += offcet["rotation"]
            tvec[0] += offcet["translation"]
            # print(rvec, " ", tvec)

            rvecs.append(rvec[0])
            tvecs.append(tvec[0])

            box_centre = np.asarray(np.mean(corners[i][0], axis=0), dtype=np.int)

            # Add different id's having different translation and rotation offcets !!!

            centre.append(self.camera.twoDto3D(box_centre) + offcet["translation"])


        
        return rvecs, tvecs, np.asarray(centre)

    def startTracking(self):
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            try:
                img = self.camera.read()
            except:
                print("Can't read image from camera")
                break

            # Detect aruco markers
            (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.aruco_type, parameters=self.arucoParams)
            if len(corners) > 0:
                rvecs, tvecs, centre = self._calculateRotationAndTranslation(corners, ids)

                self._aruco_display(corners, ids, rvecs, tvecs, img)
                closest_aruco = np.argmin(centre[:, 2])

                moving_average_rvecs = self._moving_average(rvecs[closest_aruco])
                # print(closest_aruco, " ", centre)
                print(self._radToDeg(moving_average_rvecs))
                self.sendData(self._radToDeg(moving_average_rvecs), centre[closest_aruco])

            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.flip(img, 1)
            cv2.imshow("tracking", img)

            # Press q to quit
            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
            
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

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    if args.isCamera:
        if args.videoPath is None:
            camera = FakeCamera()
        else:
            camera = FakeCamera(videoPath=args.videoPath)
    else:
        camera = AzureKinectCamera()
    
   
    objectTracker = ArucoTracker(sock, camera, args.type, ip=args.ip, port=5064)

    objectTracker.startTracking()
    
    cv2.destroyAllWindows()