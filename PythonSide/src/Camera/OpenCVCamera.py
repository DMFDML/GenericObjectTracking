
import numpy as np
from src.Camera.Camera import Camera
import cv2
from objectTrackingConstants import CHECKERBOARD
import glob

class OpenCVCamera(Camera):

    def __init__(self, camera_id = 0, calibration_files = 'images\calibration\calibrate*.png'):
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.calibration_files = calibration_files

        
    
    def read(self):
        # Output a new image from the camera
        # Input: None
        # Output: An np.array() containing the rgb image data
        _, img = self.cap.read()
        return img

    def getDepthimage(self):
        # Output the Depth image from the camera
        # Input: None
        # Output: An np.array() containing the Depth image data
        pass
        
    def getIRimage(self):
        # Output the IR image from the camera
        # Input: None
        # Output: An np.array() containing the IR image data
        pass

    
    def getPointCloud(self, bbox, mask = np.array([])):
        # return an open3d pointcloud of just the tracked object
        # Input: The bounding box for the image, Optional the mask of the object
        # Output: None
        #
        pass


    def stop(self):
        # Stop the camera
        # Input: None
        # Output: None
        self.cap.release()


    def get_calibration(self, checkerBoard = CHECKERBOARD):
        # returns the intrinsic calibration matrix of the camera
        # Input: None
        # Output: The intrinsic calibration matrix, the distortion coefficients
            
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
        objectp3d = np.zeros((1, checkerBoard[0] 
                            * checkerBoard[1], 
                            3), np.float32)
        objectp3d[0, :, :2] = np.mgrid[0:checkerBoard[0],
                                    0:checkerBoard[1]].T.reshape(-1, 2)
        prev_img_shape = None
        
        
        # Extracting path of individual image stored
        # in a given directory. Since no path is
        # specified, it will take current directory
        # jpg files alone
        images = glob.glob(self.calibration_files)
        
        for filename in images:
            image = cv2.imread(filename)
            grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
            # Find the chess board corners
            # If desired number of corners are
            # found in the image then ret = true
            ret, corners = cv2.findChessboardCorners(
                            grayColor, checkerBoard, 
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

    def twoDto3D(self, input2D):
        return [0,0,0]
