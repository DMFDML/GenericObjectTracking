import sys
sys.path.append('./src/')
import numpy as np
import cv2
import time
from Util.objectTrackingConstants import *
from Util.helperFunctions import *
import socket

from math import atan2, cos, sin, sqrt, pi
from Camera.AzureKinect import AzureKinectCamera
from Camera.FakeCamera import FakeCamera
import argparse
from ObjectTrackers.ObjectTrackerInterface import ObjectTracker
import alphashape
from pyk4a import CalibrationType
import open3d as o3d



class ReflectorTrack(ObjectTracker):

    def __init__(self, sock, camera, ip=UDP_IP, port=UDP_PORT, voxel_size = VOXEL_SIZE, colour = COLOUR, no_iterations = NO_ITERATIONS):
        self.name = "ReflectorTracker"
        self.camera = camera

        self.sock = sock
        self.ip = ip
        self.port = port

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

        self.points = None
        self.polygon = None
        self.bounding_box = None
        self.previous_rotation = np.array([0,0,0,0])
        self.previous_centre = np.array([0,0,0])

    def _getPolygon(self, ir):
        # make all the reflector points smaller (erode them) to differentiate them better
        ir = cv2.erode(ir, np.ones((3,3), np.uint8), iterations=1)
        # Detect the points
        points = self.detector.detect(ir)
        
        xys = []
        # Add points on the perimeter of the circle
        for p in points:
            xys.append(p.pt)
            for i in np.arange(0, pi * 2, pi / 4):
                x, y = p.pt
                xys.append((x + p.size * cos(i), y + p.size *sin(i)))
        
        # Only perform the alpha shape algorithm (convex hull algorithm) if there are enough data points to produce a polygon
        if len(xys) > 2:
            alpha_shape = alphashape.alphashape(xys, 0.0)
            polygon = alpha_shape.exterior.coords
            polygon = np.array(list(polygon), dtype=np.int32)
        else:
            polygon = np.array([], dtype=np.int32)
            
        return points, polygon

    # Add a new point cloud to the reference point cloud
    def _addToOriginalPointCloud(self, pcd, transform):
        # As I am using point to plane for the refining both point clouds must have normals
        if not pcd.has_normals():
            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
        # Only refine the translation and rotation if the reference point cloud has points in it
        if (len(self.original_point_cloud.points) > 0):
            # further refine the point cloud using point to plane and robust kernels
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

    def _getTranslationRotationMatrix(self, mask, bbox):
        # Get PointCloud
        pc = self.camera.getPointCloud(bbox, mask, self.colour)

        # If the point cloud has not got any points then don't perform ICP on it
        if (len(pc.points) == 0):
            return rotationMatrixToQuaternion(self.previous_matrix), self.previous_centre

        # Perform either ICP or coloured ICP between the reference point cloud and the new one
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
            # If there is an error with the ICP then return the previous values
            return rotationMatrixToQuaternion(self.previous_matrix), self.camera.getPointCloudCentre(pc)
            
        # Add the new point cloud to the original if there is enough overlap but not too much
        if (result_icp.inlier_rmse > 0.055 and result_icp.inlier_rmse < 0.3 and result_icp.fitness > 0.95):
            self._addToOriginalPointCloud(pc, result_icp.transformation)

        self.previous_matrix = result_icp.transformation
        return rotationMatrixToQuaternion(result_icp.transformation), self.camera.getPointCloudCentre(pc)
    # Add all detected points, bounding box and polygon to the image
    def _drawPolygon(self, img):         
        if len(self.polygon) > 2:
            cv2.fillPoly(img, [self.polygon], (255,0,0))
            # cv2.rectangle(img, (self.bounding_box[0],self.bounding_box[1]), (self.bounding_box[2]+self.bounding_box[0], self.bounding_box[3]+self.bounding_box[1]), (0,255,0))
        cv2.drawKeypoints(img, self.points, img, (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return img

    def trackFrame(self, img):
        self.points, self.polygon = self._getPolygon(img)

        # If the detected polygon has corners (does exist)
        if self.polygon.shape[0] > 0:
            # Calculate the minimum and maximum pixel coordinate of the polygon in order to draw a bounding box
            min_loc = self.polygon.argmin(axis = 0)
            max_loc = self.polygon.argmax(axis = 0)

            self.bounding_box = np.array([self.polygon[min_loc[0], 0], 
                self.polygon[min_loc[1], 1],
                self.polygon[max_loc[0], 0]-self.polygon[min_loc[0], 0], 
                self.polygon[max_loc[1], 1]-self.polygon[min_loc[1], 1] ])
            
            # Create a mask by setting all points insise the polygon to 1 and everything else to 0
            mask = np.zeros(img.shape, dtype=np.uint8)
            cv2.fillPoly(mask, [self.polygon], (255, 255, 255)) 
            mask = mask // 255

            # if the reference point cloud has been initialised then perform ICP otherwise create the reference pointcloud
            if (len(self.original_point_cloud.points) > 0):
                rotation, centre = self._getTranslationRotationMatrix(mask, self.bounding_box)
                rotation[2] *= -1
                self.previous_rotation, self.previous_centre = rotation, centre
                return self.previous_rotation, self.previous_centre
            else:
                self._addToOriginalPointCloud(self.camera.getPointCloud(self.bounding_box, mask, self.colour), np.identity(4))
                self.previous_centre = self.camera.getPointCloudCentre(self.original_point_cloud)
                return self.previous_rotation, self.previous_centre
        else:
            print("Could not read polygon")
            return self.previous_rotation, self.previous_centre

    def startTracking(self):  
        start = False      
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            # Read the ir image 
            try:
                img = self.camera.read()
                ir = self.camera.getIRimage().copy()

                # Threshold the image so that only the reflectors are visible
                ir_thresholded = np.asarray(ir[:, :] > 20000, dtype=np.uint8) * 255
                # turn the image into a colour image by repeating the thresholded image 3 times
                ir_colour = np.array([ir_thresholded, ir_thresholded, ir_thresholded], dtype=np.uint8).transpose(1,2,0).copy()
            except:
                print("Can't read image from camera")
                break
            
            if start:
                rotation, translation = self.trackFrame(ir_thresholded) 
                print(rotation, translation)
                sendData(rotation, translation,self.sock,self.ip, self.port)
                ir_colour = self._drawPolygon(ir_colour)


            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            # flip = cv2.flip(ir_colour, 1)
            ir_colour = cv2.putText(ir_colour, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("tracking", ir_colour)
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


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Retro-Reflector Object Tracking')
    parser.add_argument('--ip', dest='ip', default=UDP_IP, 
                    help='IP address of the computer to send data to')
    parser.add_argument('--port', dest='port', default=UDP_PORT, type=int,
                    help='Port of the computer to send data to')

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    camera = AzureKinectCamera(transformed = False, voxel_size=0.02, min_standard_deviation=1, point_cloud_threshold=1000)
    # camera = FakeCamera(transformed=False, imageFilePath="../../A_LARGE_images/images_vive_reflector/benchmark_images/")
   
    objectTracker = ReflectorTrack(sock, camera, ip=args.ip, port=args.port, voxel_size=0.02, no_iterations=30, colour=True)

    objectTracker.startTracking()
    
    cv2.destroyAllWindows()