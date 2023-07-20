import cv2
import numpy as np

import open3d as o3d
from Camera.Camera import Camera
import copy
import imageio
from Util.objectTrackingConstants import IMAGE_FILE_LOCATIONS, AZURE_CALIBRATION, DISTANCE_CONVERSION_AZURE_POINT_CLOUD, DISTANCE_CONVERSION_AZURE_DEPTH
import json


class FakeCamera(Camera):

    def __init__(self, transformed, calibration_path=AZURE_CALIBRATION, imageFilePath =IMAGE_FILE_LOCATIONS, initial_frame=0, voxel_size = 0.05, min_standard_deviation = 0.2, max_standard_deviation = 3, point_cloud_threshold = 1000, depth_scale = DISTANCE_CONVERSION_AZURE_DEPTH, point_cloud_scale = DISTANCE_CONVERSION_AZURE_POINT_CLOUD):        
        self.pcd = o3d.geometry.PointCloud()
        self.voxel_size = voxel_size
        self.min_standard_deviation = min_standard_deviation
        self.currentImage = 0
        self.point_cloud_threshold = point_cloud_threshold
        
        self.imageFilePath = imageFilePath
        self.transformed = transformed

        self.depth_scale = depth_scale
        self.point_cloud_scale = point_cloud_scale
        self.initial_frame = initial_frame
        self.max_standard_deviation = max_standard_deviation

        with open(calibration_path, 'r') as f:    
            json_data = json.load(f)
            self.matrix, self.distortion = np.array(json_data['intrinsic']), np.array(json_data['distortion'])

    def read(self):
        self.colour = np.array(imageio.imread(self.imageFilePath + "colour/colour" + str(self.initial_frame+self.currentImage) + ".tif"))
        if self.transformed:
            self.pc = np.array(imageio.imread(self.imageFilePath + "pointcloud_transform/pc" + str(self.initial_frame+self.currentImage) + ".tif"))
        else:
            self.pc = np.array(imageio.imread(self.imageFilePath + "pointcloud_no_transform/pc" + str(self.initial_frame+self.currentImage) + ".tif"))
        self.currentImage += 1

        return self.colour

    def getDepthimage(self):
        self.depth = np.array(imageio.imread(self.imageFilePath + "depth/depth" + str(self.initial_frame+self.currentImage) + ".tif"))
        return self.depth
        
    def getIRimage(self):
        self.ir = np.array(imageio.imread(self.imageFilePath + "ir/ir" + str(self.initial_frame+self.currentImage) + ".tif"))
        return self.ir

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
        
        # convert from mm to m as open3d didn't like the mm
        pc = pc / self.point_cloud_scale
        for standard_deviation_factor in np.arange(self.min_standard_deviation, self.max_standard_deviation, 0.1):
            thresholded_pc = pc[(pc[:,2] < np.mean(pc[:,2]) + standard_deviation_factor* pc[:,2].std()) & (pc[:,2] > np.mean(pc[:,2]) - standard_deviation_factor* pc[:,2].std())]
            if(thresholded_pc.shape[0] > self.point_cloud_threshold):
                self.pcd.points = o3d.utility.Vector3dVector(thresholded_pc)
                self.pcd = self.pcd.voxel_down_sample(self.voxel_size)            
                
                return self.pcd
            
        print("Not enough points to generate point cloud")
        return self.pcd
    
    def _getColourCloud(self, bbox, mask):

        pc = self.pc[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
        colour = self.pc[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2], :3].reshape(-1, 3).astype(np.float64)  
        
        if mask.size != 0:
            mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
            pc = pc[mask]
            colour = colour[mask]
        
        pc = pc / self.point_cloud_scale
        colour = colour / 255
        for standard_deviation_factor in np.arange(self.min_standard_deviation, self.max_standard_deviation, 0.1):
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
        pass

    def get_calibration(self):
        return self.matrix, self.distortion
    
    def twoDto3D(self, coordinate):        
        if not self.transformed:
            # If the user wants the transformed depth point cloud or not
            print(self.pc[coordinate[1]][coordinate[0]] / self.depth_scale)
            return self.pc[coordinate[1]][coordinate[0]] / self.depth_scale
        else:
            raise Exception("Not implemented for non transformed point cloud")

    def getPointCloudCentre(self, pc):
        return pc.get_center() / (self.depth_scale / self.point_cloud_scale)
