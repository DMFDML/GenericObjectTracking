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
import alphashape
from pyk4a import CalibrationType
import open3d as o3d



class ReflectorTrack(ObjectTracker):

    def __init__(self, sock, camera, ip=UDP_IP, port=UDP_PORT, voxel_size = VOXEL_SIZE, colour = COLOUR, no_iterations = NO_ITERATIONS):
        self.camera = camera

        self.sock = sock
        self.ip = ip
        self.port = port

        self.moving_average_rotation = [(0,0,0)]
        # self.moving_average_translation = [(0,0,0)]
        self.moving_average_length = 10
        self.moving_average_weights = np.array([0.2,0.2,0.4,0.4,0.6,0.6,0.8,0.8,1,1])
        # Set up the detector with default parameters.
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.blobColor = 255
        params.minThreshold = 200
        params.maxThreshold = 256

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 5000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.5

        self.detector = cv2.SimpleBlobDetector_create(params)

        self.previous_matrix = np.identity(4)
        self.previous_centre = np.array([0,0,0])
        self.original_point_cloud = o3d.geometry.PointCloud()
        self.voxel_size = voxel_size
        self.colour = colour
        self.no_iterations = no_iterations
        self.save = 0

        self.points = None
        self.polygon = None
        self.bounding_box = None
        self.previous_rotation = np.array([0,0,0])
        self.previous_centre = np.array([0,0,0])
       
    def _movingAverage(self, new_value):
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

    def _getPolygon(self, ir):
        ir = cv2.erode(ir, np.ones((3,3), np.uint8), iterations=1)
        points = self.detector.detect(ir)
        
        xys = []
        # Add points on the perimeter of the circle
        for p in points:
            xys.append(p.pt)
            for i in np.arange(0, pi * 2, pi / 4):
                x, y = p.pt
                xys.append((x + p.size * cos(i), y + p.size *sin(i)))
        
        if len(xys) > 2:
            alpha_shape = alphashape.alphashape(xys, 0.0)
            polygon = alpha_shape.exterior.coords
            polygon = np.array(list(polygon), dtype=np.int32)
        else:
            polygon = np.array([], dtype=np.int32)
            
        return points, polygon

    def _addToOriginalPointCloud(self, pcd, transform):
        if not pcd.has_normals():
            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))

        if (np.asarray(self.original_point_cloud.points).shape[0] > 0):
            loss = o3d.pipelines.registration.TukeyLoss(k=0.1)
            p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
            result_icp = o3d.pipelines.registration.registration_icp(pcd, self.original_point_cloud, 0.1, transform,
                                                                p2l, 
                                                                o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                    relative_rmse=1e-6,
                                                                    max_iteration=150))
            # If the fitness isn't good enough then don't add the points
            if (result_icp.fitness < 0.8):
                return
            pcd.transform(result_icp.transformation)
            print(result_icp.fitness, " ", result_icp.inlier_rmse)
        self.original_point_cloud += pcd
        self.original_point_cloud = self.original_point_cloud.voxel_down_sample(self.voxel_size)
        self.original_point_cloud.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))

        print("Adding POINT CLOUD ", np.asarray(self.original_point_cloud.points).shape[0])
        o3d.io.write_point_cloud("./images/pc" + str(self.save)+".ply", self.original_point_cloud)
        self.save += 1

    def _getTranslationRotationMatrix(self, mask, bbox):
        # Get PointCloud
        pc = self.camera.getPointCloud(bbox, mask, self.colour)

        if (np.asarray(pc.points).shape[0] == 0):
            return self._rotationMatrixToQuaternion(self.previous_matrix), self.previous_centre
            # return self.previous_matrix, self.previous_centre

        # Perform ICP on the originaly generated point cloud and the new one
        try:
            if (self.colour):
                result_icp = o3d.pipelines.registration.registration_colored_icp(
                    pc, self.original_point_cloud, 1.0, self.previous_matrix,
                    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                relative_rmse=1e-6,
                                                                max_iteration=self.no_iterations))
            else:
                result_icp = o3d.pipelines.registration.registration_icp(
                    pc, self.original_point_cloud, 1.0, self.previous_matrix,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                relative_rmse=1e-6,
                                                                max_iteration=self.no_iterations))
        except(RuntimeError):
            return self._rotationMatrixToQuaternion(self.previous_matrix), pc.get_center()
            # return self.previous_matrix, pc.get_center()
        print(result_icp.inlier_rmse, result_icp.fitness )
        # Add the new point cloud to the original if there is enough overlap but not too much
        if (result_icp.inlier_rmse > 0.055 and result_icp.inlier_rmse < 0.3 and result_icp.fitness > 0.95):
        
            self._addToOriginalPointCloud(pc, result_icp.transformation)

        self.previous_matrix, self.previous_centre = result_icp.transformation, pc.get_center()
        return self._rotationMatrixToQuaternion(result_icp.transformation), pc.get_center()
        # return result_icp.transformation, pc.get_center()

    def _drawPolygon(self, img): 
        if len(img.shape) > 2:
            img = self.camera.capture.transformed_color.copy()
        
        if len(self.polygon) > 2:
            cv2.fillPoly(img, [self.polygon], (255,0,0))
            cv2.rectangle(img, (self.bounding_box[0],self.bounding_box[1]), (self.bounding_box[2]+self.bounding_box[0], self.bounding_box[3]+self.bounding_box[1]), (0,255,0))
        cv2.drawKeypoints(img, self.points, img, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return img

    def _trackFrame(self, img):
        self.points, self.polygon = self._getPolygon(img)
    
        if self.polygon.shape[0] > 0:
            min_loc = self.polygon.argmin(axis = 0)
            max_loc = self.polygon.argmax(axis = 0)
            
            self.bounding_box = np.array([self.polygon[min_loc[0], 0], 
                self.polygon[min_loc[1], 1],
                self.polygon[max_loc[0], 0]-self.polygon[min_loc[0], 0], 
                self.polygon[max_loc[1], 1]-self.polygon[min_loc[1], 1] ])
            
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [self.polygon], (255, 255, 255)) 
            mask = mask // 255

            if (len(self.original_point_cloud.points) > 0):
                rotation, centre = self._getTranslationRotationMatrix(mask, self.bounding_box)
                self.previous_rotation, self.previous_centre = rotation, centre
                return self.previous_rotation, self.previous_centre
            else:
                self._addToOriginalPointCloud(self.camera.getPointCloud(self.bounding_box, mask, self.colour), np.identity(4))
                self.previous_centre = self.original_point_cloud.get_center()
                return self.previous_rotation, self.previous_centre
        else:
            print("Could not read point cloud")
            return self.previous_rotation, self.previous_centre

    def startTracking(self):  
        start = False      
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            try:
                img = self.camera.read()
                ir = self.camera.getIRimage()
            except:
                print("Can't read image from camera")
                break

            ir_thresholded = np.asarray(ir[:, :] > 20000, dtype=np.uint8) * 255
            ir_colour = self.camera.capture.transformed_color.copy()
            
            if start:
                rotation, translation = self._trackFrame(ir_thresholded) 
                self.sendData(rotation, translation)
                ir_colour = self._drawPolygon(ir_colour)


            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            flip = cv2.flip(ir_colour, 1)
            flip = cv2.putText(flip, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("tracking", flip)
            # cv2.imshow("mask", mask * 255)

            # Press q to quit
            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('o'):
                try:
                    print("saving original point cloud")
                    o3d.io.write_point_cloud("./images/original.ply", self.original_point_cloud)
                except(UnboundLocalError):
                    print("PROBLEM SAVING") 
            elif k == ord('s'):
                self.original_point_cloud = o3d.geometry.PointCloud()
                start = not start

            
        self.camera.stop()

    def benchMark(self):
        pass

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Retro-Reflector Object Tracking')
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
        camera = AzureKinectCamera(transformed = False, voxel_size=0.03, min_standard_deviation=1, point_cloud_threshold=2000)
    
   
    objectTracker = ReflectorTrack(sock, camera, ip=args.ip, port=args.port, voxel_size=0.05, no_iterations=30, colour=True)

    objectTracker.startTracking()
    
    cv2.destroyAllWindows()