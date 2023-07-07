import cv2
from SiamMask import SiamMaskSharpTracker
from Re3 import Re3Tracker
import socket
import json
import math
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from AzureKinect import AzureKinectCamera
import open3d as o3d

segmentation = 0
TRACKER = "SiamMask"


class ObjectTracker:
    def __init__(self, sock, tracker, camera_no = 0, padding = 20):
        self.tracker = tracker

        self.camera = AzureKinectCamera()

        self.sock = sock
        self.padding = padding
        self.smoothing_weights = np.array([1.0, 0.5, 0.3,0.2,0.1,0.1,0.05]) 
        self.number_of_smoothing_frames = len(self.smoothing_weights)
        self.previous_angles = np.zeros(self.number_of_smoothing_frames)
        self.box = None
        self.save = 0

    # Sends data to unity
    def sendData(self, img, box, rotation):
        x,y,w,h = box
        img_height, img_width, _ = img.shape
        # print(img.shape, " ", x, " ", y, " ", w, " ", h)
        data = {
            "box": {
                "x": x / img_width,
                "y": y / img_height,
                "w": w / img_width,
                "h": h / img_height,
                "r": rotation
            },
        }

        packet = json.dumps(data, indent=3)
        self.sock.sendto( (packet).encode(), (UDP_IP, UDP_PORT) )

    # Prompts the user to draw a bounding box around the object to be tracked
    def captureBox(self):
        img = self.camera.read()

        box = cv2.selectROI("tracking", img, False)
        self.tracker.start(img, box)
        self.box = box

    # Return a srunk image of just the bounding box
    def getOnlyBox(self, img, box):
        # add padding to the coordinates of the box
        x1, y1, x2, y2 = int(box[0]-self.padding), int(box[1]-self.padding), int(box[0]+box[2]+self.padding), int(box[1]+box[3]+self.padding)
        # make sure the box is within the image
        x1, y1, x2, y2 = min(max(x1, 0), img.shape[1]), min(max(y1, 0), img.shape[0]), max(min(x2, img.shape[1]), 0), max(min(y2, img.shape[0]), 0)
        # return the shrunk image
        return img[y1:y2, x1:x2]

        ## [pca]
        # Construct a buffer used by the pca analysis
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
        
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        
        # Store the center of the object
        cntr = (int(mean[0,0] + box[0]-self.padding), int(mean[0,1] + box[1]-self.padding))
        ## [pca]
        
        ## [visualization]
        # Draw the principal components
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        self.drawAxis(img, cntr, p1, (255, 255, 0), 1)
        self.drawAxis(img, cntr, p2, (0, 0, 255), 5)
        
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]
        
        # Label with the rotation angle
        label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
        textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        
        return angle

    def displayMinimumBoundingBox(self, img, min_box, tracked_box):
        x1,y1 = int(min_box[0,0]+ tracked_box[0]-self.padding), int(min_box[0,1]+ tracked_box[1]-self.padding)
        x2,y2 = int(min_box[1,0]+ tracked_box[0]-self.padding), int(min_box[1,1]+ tracked_box[1]-self.padding)
        x3,y3 = int(min_box[2,0]+ tracked_box[0]-self.padding), int(min_box[2,1]+ tracked_box[1]-self.padding)
        x4,y4 = int(min_box[3,0]+ tracked_box[0]-self.padding), int(min_box[3,1]+ tracked_box[1]-self.padding)
        b = np.array([[x1,y1],[x2,y2],[x3,y3],[x4,y4]], dtype=np.int32)

        cv2.drawContours(img,[b],0,(0,255,255),2)  
    
    # def smoothing(self, angle):
    #     # Move all the previous angles down one and add the newly calculated angle
    #     self.previous_angles = np.roll(self.previous_angles, 1)
    #     self.previous_angles[0] = angle

    #     # Calculate the weighted average of the previous angles and updated the previously calcuated angles
    #     avg = np.sum(self.smoothing_weights * self.previous_angles) / np.sum(self.smoothing_weights)
    #     self.previous_angles[0] = avg

    #     return avg
 
    # def calculateAngle_PCA(self, contour):
    #     # Taken from https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/

    #     # Construct a buffer used by the pca analysis
    #     sz = len(contour)
    #     data_pts = np.empty((sz, 2), dtype=np.float64)
    #     for i in range(data_pts.shape[0]):
    #         data_pts[i,0] = contour[i,0,0]
    #         data_pts[i,1] = contour[i,0,1]
        
    #     # Perform PCA analysis
    #     mean = np.empty((0))
    #     mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        
    #     # Calculate the centre of the object
    #     cntr = (int(mean[0,0]), int(mean[0,1]))
    #     # Calculate the angle of the object based off the eigenvectors
    #     angle = atan2(eigenvectors[0,1], eigenvectors[0,0])
        
    #     return cntr, -int(np.rad2deg(angle))

    # def calculateAngle_Box(self, contour):
    #     # Calculate the minimum area bounding box
    #     rect = cv2.minAreaRect(contour)
    #     # Retrieve the key parameters of the rotated bounding box
    #     centre = (int(rect[0][0]),int(rect[0][1])) 
    #     width = int(rect[1][0])
    #     height = int(rect[1][1])
    #     angle = int(rect[2])
        
    #     # If the orientation is portrait, then the angle is 90 degrees off
    #     if width < height:
    #         angle = 90 - angle
    #     else:
    #         angle = -angle
        
    #     return centre, angle

    # def getRotations(self, img, box):
    #     # change the colour of the image captured by the bounding box
    #     small_image = cv2.cvtColor(self.getOnlyBox(img,box), cv2.COLOR_BGR2GRAY)
    #     cv2.imshow("small1", small_image)
    #     # Apply mask to image if it has a mask to get a better outline (e.g. Siammask)
    #     if not self.tracker.getMask() is None:
    #         small_mask = self.getOnlyBox(self.tracker.getMask(), box)
    #         small_image = np.uint8(small_mask * 255)
            
    #     else: 
    #         # Convert image to binary
    #         # small_image = cv2.adaptiveThreshold(small_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
    #         ret,small_image = cv2.threshold(small_image,127,255,cv2.THRESH_BINARY_INV)
            
    #     cv2.imshow("small2", small_image)
    #     # Find the contours in the thresholded image
    #     _, contours, _ = cv2.findContours(small_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #     contour = max(contours, key=cv2.contourArea)
    #     cv2.drawContours(self.getOnlyBox(img,box), contour, -1, (0,0,255), 3)

    #     # Calculate the centre and angle of the object using either PCA or the minimum bounding box
    #     centre1, angle1 = self.calculateAngle_PCA(contour)
    #     centre2, angle2 = self.calculateAngle_Box(contour)

    #     # Smooth the angle using a weighted average of the previous angles
    #     angle = self.smoothing(angle1)
        
    #     # Display the angle and centres of the object
    #     label = "  Rotation Angle: " + str(angle) + " degrees"
    #     cv2.putText(img, label, (150,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 1, cv2.LINE_AA)
    #     centre1 = (int(centre1[0] + box[0] - self.padding), int(centre1[1] + box[1] - self.padding))
    #     centre2 = (int(centre2[0] + box[0] - self.padding), int(centre2[1] + box[1] - self.padding))
    #     cv2.circle(img, centre1, 3, (255, 0, 255), 2)
    #     cv2.circle(img, centre2, 3, (0, 255, 255), 2)
    #     self.displayMinimumBoundingBox(img, cv2.boxPoints(cv2.minAreaRect(contour)), box)

    #     return angle

    def startTracking(self):
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            img = self.camera.read()

            # Only track object if a bounding box has been selected
            if (not self.box == None):

                new_box = self.tracker.update(img)

                if (not new_box == []):
                    # rotation = self.getRotations(img, new_box)
                    self.sendData(img, new_box, 0.0)
                    
                tracker.drawBox(img)

            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.flip(img, 1)
            cv2.imshow("tracking", img)

            # Press q to quit and d to select a new bounding box
            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('d') :
                print("Captureing box")
                self.captureBox()
            elif k & 0xFF == ord('s'):
                print("saving image")
                o3d.io.write_point_cloud("./images/pc" + str(self.save)+".ply", self.camera.getPointCloud(new_box, self.tracker.getMask()))
                cv2.imwrite("./images/pc" + str(self.save)+".png", img)
                self.save += 1

        self.camera.stop()

if __name__ == "__main__":
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5065
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    tracker = None
    if TRACKER == "SiamMask":
        tracker = SiamMaskSharpTracker()
    elif TRACKER == "Re3":
        tracker = Re3Tracker()
    
    if not tracker is None:
        objectTracker = ObjectTracker(sock, tracker)
        # objectTracker.captureBox()
        objectTracker.startTracking()
        cv2.destroyAllWindows()