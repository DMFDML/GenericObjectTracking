import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration, CalibrationType
import open3d as o3d
from src.Camera.Camera import Camera
import copy


class AzureKinectCamera(Camera):

    def __init__(self, voxel_size = 0.05, min_standard_deviation = 0.2, point_cloud_threshold = 1000, transformed = True):
        self.k4a = PyK4A(
            Config(
                color_resolution=pyk4a.ColorResolution.RES_720P,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            )
        )

        self.k4a.start()


        # getters and setters directly get and set on device
        self.k4a.whitebalance = 4500
        assert self.k4a.whitebalance == 4500
        self.k4a.whitebalance = 4510
        assert self.k4a.whitebalance == 4510

        self.pcd = o3d.geometry.PointCloud()
        self.voxel_size = voxel_size
        self.min_standard_deviation = min_standard_deviation
        self.transformed = transformed
        self.point_cloud_threshold = point_cloud_threshold

    def read(self):
        self.capture = self.k4a.get_capture()

        if not np.any(self.capture.color):
            raise Exception("Camera capture colour not working")

        return self.capture.color[:, :, :3].copy()

    def getDepthimage(self):
        return self.capture.depth[:, :].copy()
        
    def getIRimage(self):
        return self.capture.ir[:, :].copy()

    def getPointCloud(self, bbox, mask = np.array([]), colour = False):
        if colour:
            return self._getColourCloud(bbox, mask)
        else:
            return self._getNoColourCloud(bbox, mask)

    def _getNoColourCloud(self, bbox, mask):
        if self.transformed:
            # If the user wants the transformed depth point cloud or not
            pc = self.capture.transformed_depth_point_cloud[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
        else:
            pc = self.capture.depth_point_cloud[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
            
        if mask.size != 0:
            mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
            pc = pc[mask]
        
        # convert from cm to mm as open3d didn't like the mm
        pc = pc / 100
        
        for standard_deviation_factor in np.arange(self.min_standard_deviation, 1, 0.1):
            thresholded_pc = pc[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]
            if(thresholded_pc.shape[0] > self.point_cloud_threshold):
                self.pcd.points = o3d.utility.Vector3dVector(thresholded_pc)
                self.pcd = self.pcd.voxel_down_sample(self.voxel_size)            
                
                return self.pcd
            
        print("Not enough points to generate point cloud")
        return self.pcd
    
    def _getColourCloud(self, bbox, mask):
        if self.transformed:
            # If the user wants the transformed depth point cloud or not
            pc = self.capture.transformed_depth_point_cloud[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
            colour = self.capture.color[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2], :3].reshape(-1, 3).astype(np.float64)  
        else:
            pc = self.capture.depth_point_cloud[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
            colour = self.capture.transformed_color[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2], :3].reshape(-1, 3).astype(np.float64)  
        
        if mask.size != 0:
            mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
            pc = pc[mask]
            colour = colour[mask]
        
        pc = pc / 100
        colour = colour / 255
        
        for standard_deviation_factor in np.arange(self.min_standard_deviation, 2, 0.1):
            thresholded_pc = pc[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]
            thresholded_colour = colour[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]
            
            if(thresholded_pc.shape[0] > self.point_cloud_threshold):
                self.pcd.points = o3d.utility.Vector3dVector(thresholded_pc)
                self.pcd.colors = o3d.utility.Vector3dVector(thresholded_colour)
                
                # Voxelise point cloud
                self.pcd = self.pcd.voxel_down_sample(self.voxel_size)

                # Generate Normals
                self.pcd.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
                
                return self.pcd
            
        print("Not enough points to generate point cloud")
        return self.pcd

    def stop(self):
        self.k4a.stop()

    def get_calibration(self):
        print(self.k4a.calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.COLOR), " ", self.k4a.calibration.get_distortion_coefficients(pyk4a.calibration.CalibrationType.COLOR))
        return self.k4a.calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.COLOR), self.k4a.calibration.get_distortion_coefficients(pyk4a.calibration.CalibrationType.COLOR)
    
    def twoDto3D(self, coordinate):
        # print(coordinate, self.capture.transformed_depth_point_cloud.shape)
        
        if self.transformed:
            # If the user wants the transformed depth point cloud or not
            return self.capture.transformed_depth_point_cloud[coordinate[1]][coordinate[0]] / 100
        else:
            return self.capture.depth_point_cloud[coordinate[1]][coordinate[0]] / 100
