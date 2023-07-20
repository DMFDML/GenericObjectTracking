from abc import ABC, abstractmethod
import numpy as np

class Camera(ABC):


    @abstractmethod
    def read(self):
        # Output a new image from the camera
        # Input: None
        # Output: An np.array() containing the rgb image data
        pass

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

    @abstractmethod
    def getPointCloud(self, bbox, mask = np.array([])):
        # return an open3d pointcloud of just the tracked object
        # Input: The bounding box for the image, Optional the mask of the object
        # Output: None
        #
        pass

    @abstractmethod
    def stop(self):
        # Stop the camera
        # Input: None
        # Output: None
        pass

    @abstractmethod
    def get_calibration(self):
        # returns the intrinsic calibration matrix of the camera
        # Input: None
        # Output: The intrinsic calibration matrix, the distortion coefficients
        pass

    @abstractmethod
    def getPointCloudCentre(self, pc):
        # returns the centre of the point cloud in m
        pass



