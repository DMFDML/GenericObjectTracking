import cv2
from src.Trackers.SiamMask import SiamMaskSharpTracker
from src.Trackers.Re3 import Re3Tracker
import socket
import json
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from src.Camera.AzureKinect import AzureKinectCamera
from src.Camera.FakeCamera import FakeCamera
import open3d as o3d
import time
import argparse
from ObjectTrackerInterface import ObjectTracker

from objectTrackingConstants import *

class GenericObjectTracker(ObjectTracker):
    def __init__(self, sock, tracker, camera, ip=UDP_IP, port=UDP_PORT, voxel_size = VOXEL_SIZE, colour = COLOUR, no_iterations = NO_ITERATIONS, box = []):
        self.tracker = tracker

        self.camera = camera

        self.sock = sock

        self.previous_matrix = np.identity(4)
        self.previous_centre = np.array([0,0,0])
        self.original_point_cloud = o3d.geometry.PointCloud()
        self.voxel_size = voxel_size
        self.colour = colour
        self.no_iterations = no_iterations
        self.box = box
        self.ip = ip
        self.port = port
        if box != []:
            self._startTrackingBox(box)  

        self.previous_rotation = np.array([0,0,0])   

    # Prompts the user to draw a bounding box around the object to be tracked
    def _startTrackingBox(self, box = []):
        img = self.camera.read()

        # Prompt the user to draw a bounding box around the object to be tracked
        if len(box) == 0:
            box = cv2.selectROI("tracking", img, False)

        self.tracker.start(img, box)
        self.box = box
        self.tracker.update(img)
        self.original_point_cloud.clear()
        
        self._addToOriginalPointCloud(self.camera.getPointCloud(box, self.tracker.getMask(), self.colour), np.identity(4))
    
    # Return a srunk image of just the bounding box
    def _getOnlyBox(self, img, box):
        # get the coordinates of the box
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[0]+box[2]), int(box[1]+box[3])
        # make sure the box is within the image
        x1, y1, x2, y2 = min(max(x1, 0), img.shape[1]), min(max(y1, 0), img.shape[0]), max(min(x2, img.shape[1]), 0), max(min(y2, img.shape[0]), 0)
        # return the shrunk image
        return img[y1:y2, x1:x2]

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

        # Add to the original point cloud, voxelise it to reduce the amount of overlap and estimate the normals
        self.original_point_cloud += pcd
        self.original_point_cloud = self.original_point_cloud.voxel_down_sample(self.voxel_size)
        self.original_point_cloud.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))

        print("Adding POINT CLOUD ", np.asarray(self.original_point_cloud.points).shape[0])

    def _getTranslationRotationMatrix(self, bbox):
        # Get PointCloud
        pc = self.camera.getPointCloud(bbox, self.tracker.getMask(), self.colour)

        # If the point cloud has not got any points then don't perform ICP on it
        if (len(pc.points) == 0):
            return self._rotationMatrixToQuaternion(self.previous_matrix), self.previous_centre
        

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
            return self._rotationMatrixToQuaternion(self.previous_matrix), self.previous_centre

        # Add the new point cloud to the original if there is enough overlap but not too much
        if (result_icp.inlier_rmse > 0.08 and result_icp.inlier_rmse < 0.18 and result_icp.fitness > 0.95):
            self._addToOriginalPointCloud(pc, result_icp.transformation)

        self.previous_matrix, self.previous_centre = result_icp.transformation, pc.get_center()
        return self._rotationMatrixToQuaternion(result_icp.transformation), pc.get_center()

    # Track the object for a frame
    def _trackFrame(self, img):
        new_box = self.tracker.update(img)

        if (len(new_box) == 4):
            rotation, centre = self._getTranslationRotationMatrix(new_box)
            self.previous_rotation = rotation
            return rotation, centre
        else:
            return self.previous_rotation, self.previous_centre
        
    def startTracking(self):
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            # Read image
            try:
                img = self.camera.read()
            except:
                print("Can't read image from camera")
                break

            # Only track object if a bounding box has been selected
            if (len(self.box) == 4):
                rotation, centre = self._trackFrame(img)
                self.sendData(rotation, centre)
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
            # press s to start the tracking
            elif k  == ord('s') :
                print("Captureing box")
                self._startTrackingBox()
            # press o to save the reference pointcloud
            elif k & 0xFF == ord('o'):
                try:
                    print("saving original point cloud")
                    # o3d.visualization.draw_geometries([self.original_point_cloud])
                    o3d.io.write_point_cloud("./images/original.ply", self.original_point_cloud)
                except(UnboundLocalError):
                    print("PROBLEM SAVING") 
        self.camera.stop()

    def benchMark(self, speed_file, translation_rotation):
        speed_file.write("Frame, Total FPS, Tracker Time, Translation Rotation Time\n")
        translation_rotation.write("Frame, Rotation, Translation\n")
        frame = 0
        while True:
            # Start timer for fps calculation
            try:
                img = self.camera.read()
            except:
                print("Out of Images")
                break
            
            start = time.time()

            # Only track object if a bounding box has been selected
            if (not self.box == []):
                tracker_start = time.time()
                new_box = self.tracker.update(img)
                tracker_end = time.time()
                if (not new_box == []):
                    rotation_start = time.time()
                    rotation, centre = self._getTranslationRotationMatrix(new_box)
                    
                    # Calculate the fps and write to file
                    speed_file.write(str(frame) + ", " + str(1/(time.time() - start)) + ", " + str(tracker_end - tracker_start) + ", " + str(time.time() - rotation_start) + ", \n")
                    # Calculate the rotation and translation and send to file
                    translation_rotation.write(str(frame) + ", " + str(rotation) + ", " + str(centre) + ", \n")

            print("Processed Frame: " + str(frame))
            frame += 1
            

        self.camera.stop()
        speed_file.write("\n\n")
        translation_rotation.write("\n\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generic Object Tracking')
    parser.add_argument('--tracker', dest='tracker', default=TRACKER,
                    help='Type of tracker being used')
    parser.add_argument('--ip', dest='ip', default=UDP_IP,
                    help='IP address of the computer to send data to')
    parser.add_argument('--port', dest='port', default=UDP_PORT,
                    help='Port of the computer to send data to')
    parser.add_argument('--isCamera', dest='isCamera', default=FAKE_CAMERA, type=bool,
                    help='If a camera is being used or not')
    parser.add_argument('--videoPath', dest='videoPath', default=None,
                    help='The path to the images being used')
    parser.add_argument('--voxel_size', dest='voxel_size', default=VOXEL_SIZE,
                    help='The size of the voxels being used')
    parser.add_argument('--colour', dest='colour', type=bool, default=COLOUR,
                    help='If colour ICP is being used or not')
    parser.add_argument('--no_iterations', dest='no_iterations', default=NO_ITERATIONS,
                    help='The number of iterations for ICP')
    parser.add_argument('--box', dest='box', default=[],
                    help='The location of the bounding box')

    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.tracker == "SiamMask":
        tracker = SiamMaskSharpTracker()
    elif args.tracker == "Re3":
        tracker = Re3Tracker()
    else:
        raise ValueError("Invalid tracker type")

    if args.isCamera:
        if args.videoPath is None:
            camera = FakeCamera(voxel_size=args.voxel_size)
        else:
            camera = FakeCamera(voxel_size=args.voxel_size, videoPath=args.videoPath)
    else:
        camera = AzureKinectCamera(voxel_size=args.voxel_size)
    
    if not tracker is None:
        objectTracker = GenericObjectTracker(sock, tracker, camera, ip=args.ip, port=args.port, voxel_size=args.voxel_size, colour=args.colour, no_iterations=args.no_iterations, box=args.box)

        objectTracker.startTracking()
        
        cv2.destroyAllWindows()