import cv2
import numpy as np

import open3d as o3d
from src.Camera.Camera import Camera
import copy
import imageio
from objectTrackingConstants import IMAGE_FILE_LOCATIONS
import json

class FakeCamera(Camera):

    def __init__(self, imageFilePath =IMAGE_FILE_LOCATIONS,  voxel_size = 0.05, min_standard_deviation = 0.2):

        
        self.pcd = o3d.geometry.PointCloud()
        self.voxel_size = voxel_size
        self.min_standard_deviation = min_standard_deviation
        self.currentImage = 0
        calibration = open("./src/Camera/calibration.json", "r")
        self.calibration = json.load(calibration.read())
        calibration.close()
        
        self.imageFilePath = imageFilePath

    def read(self):
        self.colour = imageio.imread(self.imageFilePath + "colours/colour" + str(self.currentImage) + ".jpg")
        self.pc = imageio.imread(self.imageFilePath + "pcds/pc" + str(self.currentImage) + ".tif")
        self.currentImage += 1

        return self.colour

    def getDepthimage(self):
        return imageio.imread(self.imageFilePath + "colours/depth" + str(self.currentImage) + ".jpg")
        
    def getIRimage(self):
        return imageio.imread(self.imageFilePath + "colours/ir" + str(self.currentImage) + ".jpg")

    def getPointCloud(self, bbox, mask = np.array([]), colour = False):
        if colour:
            return self._getColourCloud(bbox, mask)
        else:
            return self._getNoColourCloud(bbox, mask)

    def _getNoColourCloud(self, bbox, mask):

        pc = self.pc[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
        
        if mask.size != 0:
            mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
            pc = pc[mask]
        
        # convert from cm to mm as open3d didn't like the mm
        pc = pc / 100
        
        for standard_deviation_factor in np.arange(self.min_standard_deviation, 1, 0.1):
            thresholded_pc = pc[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]

            if(thresholded_pc.shape[0] > 1000):
                self.pcd.points = o3d.utility.Vector3dVector(thresholded_pc)
                self.pcd = self.pcd.voxel_down_sample(self.voxel_size)            

                return self.pcd
            
        print("Not enough points for point cloud")
        return self.pcd
    
    def _getColourCloud(self, bbox, mask):
        
        pc = self.pc[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
        colour = self.colour[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2], :3].reshape(-1, 3).astype(np.float64)  
        if mask.size != 0:
            mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
            pc = pc[mask]
            colour = colour[mask]
        
        pc = pc / 100
        colour = colour / 255
        
        for standard_deviation_factor in np.arange(self.min_standard_deviation, 1, 0.1):
            thresholded_pc = pc[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]
            thresholded_colour = colour[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]

            if(thresholded_pc.shape[0] > 1000):
                self.pcd.points = o3d.utility.Vector3dVector(thresholded_pc)
                self.pcd.colors = o3d.utility.Vector3dVector(thresholded_colour)
                
                # Voxelise point cloud
                self.pcd = self.pcd.voxel_down_sample(self.voxel_size)

                # Generate Normals
                self.pcd.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
            
        print("Not enough points for point cloud")
        return self.pcd

    def stop(self):
        pass

    def get_calibration(self):
        print("NEED TO IMPLEMENT CALIBRATION CODE !!!!")
        raise NotImplementedError
        return None, None
    
    
