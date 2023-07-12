import sys
# sys.path.append("./sort_oh")
import numpy as np
import cv2
import time
from objectTrackingConstants import *
import socket

from math import atan2, cos, acos, sin, sqrt, pi, asin
from src.Camera.AzureKinect import AzureKinectCamera
from src.Camera.FakeCamera import FakeCamera
import argparse
from ObjectTrackerInterface import ObjectTracker
import alphashape
from pyk4a import CalibrationType
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D




class ReflectorTrackNoICP(ObjectTracker):

    def __init__(self, sock, camera, ip=UDP_IP, port=UDP_PORT):
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
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 5000

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.25

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.25

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.25

        self.detector = cv2.SimpleBlobDetector_create(params)
        self.initial_rotation = None

        self.points = []
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
    
    def _getPolygon(self, points):
        
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
            
        return polygon

    def _getPointsAndClusters(self, ir, depth):
        dialated = cv2.dilate(ir, np.ones((5,5), np.uint8), iterations=3)
        eroded = cv2.erode(ir, np.ones((3,3), np.uint8), iterations=2)
        
        points_dialated = self.detector.detect(dialated)
        points_eroded = self.detector.detect(eroded)

        # change points to in a dict of shape [{centre: point, points: [point]}]
        points = []
        for c in points_dialated:
            points.append({'centre': c, 'points': [], 'point_xyz': [], 'centre_xyz': []})
            
            # Add points on the outer perimeter of the cluster
            for i in np.arange(0, pi * 2, pi / 4):
                # get the x,y coordinates of the point on the perimeter 1.9 is used instead of 2 as we want it just outside the circle
                x, y = (c.pt[0] + c.size/1.5 * cos(i), c.pt[1] + c.size/1.5 *sin(i))
                x,y = min(max(x, 0), depth.shape[1]-1), min(max(y, 0), depth.shape[0]-1)
                z = depth[int(y), int(x)]
                if z > 0 and z < 1000:
                    points[-1]['centre_xyz'].append((x, y, z))
            
            # If there are no valid points on the perimeter of the cluster don't add any interior points
            if len(points[-1]['centre_xyz']) > 0:
                # Add points within the circle
                for p in points_eroded:
                    # If point is within the cluster circle
                    if c.pt[0] + c.size > p.pt[0] > c.pt[0] - c.size and c.pt[1] + c.size > p.pt[1] > c.pt[1] - c.size:
                        points[-1]['points'].append(p)
                        # Add points on the outer perimeter of the circle due to the centre not having a depth measurement
                        for i in np.arange(0, pi * 2, pi / 4):
                            # get the x,y coordinates of the point on the perimeter 1.9 is used instead of 2 as we want it just outside the circle
                            x, y = (p.pt[0] + p.size/1.5 * cos(i), p.pt[1] + p.size/1.5 *sin(i))
                            x,y = min(max(x, 0), depth.shape[1]-1), min(max(y, 0), depth.shape[0]-1)
                            z = depth[int(y), int(x)]
                            if z > 0:
                                points[-1]['point_xyz'].append((x, y, z))
            else:
                points.pop()
            
                
        return points

    def _getPlanes(self, points):
        planes = []
        centre_xyzs = []
        for c in points:
            point_xyzs = np.array(c['point_xyz'])
            if len(point_xyzs) > 3:
                xys, zs = np.c_[point_xyzs[:,0], point_xyzs[:,1], np.ones(point_xyzs.shape[0])], point_xyzs[:, 2]
                plane_params, residuals, _, _ = np.linalg.lstsq(xys, zs.T, rcond=None)
                normal = np.array([plane_params[0], plane_params[1], -1])
                planes.append({'centre': c['centre'], 'fit': plane_params, 'residual': residuals, 'normal': normal / np.linalg.norm(normal)})

            centre_xyzs += c['centre_xyz']

        centre_xyzs = np.asarray(centre_xyzs).reshape(-1, 3)
        centre_plane = {}
        if len(centre_xyzs) > 3:
            xys, zs = np.c_[centre_xyzs[:,0], centre_xyzs[:,1], np.ones(centre_xyzs.shape[0])], centre_xyzs[:, 2]
            centre_params, residuals, _, _ = np.linalg.lstsq(xys, zs.T, rcond=None)
            normal = np.array([centre_params[0], centre_params[1], -1])
            centre_plane = {'fit': centre_params , 'residual': residuals, 'normal': normal / np.linalg.norm(normal)}
        
        return planes, centre_plane

    def _getCentre(self, points):
        total = np.zeros(3)
        amount = 0
        for c in points:
            centre_xyz = np.array(c['centre_xyz']).reshape(-1, 3)
            point_xyz = np.array(c['point_xyz']).reshape(-1, 3)
            total += centre_xyz.sum(axis=0)
            total += point_xyz.sum(axis=0)
            amount += len(centre_xyz) + len(point_xyz)
        
        return total / amount

    def _getRotation(self, planes, centre_plane):
        normals = []
        if len(planes) == 0:
            normals = [p['normal'] for p in planes]
        
        if len(centre_plane) > 0:
            normals.append(centre_plane['normal'])

        if len(normals) == 0:
            return np.array([0, 0, 0])
        

        normals = np.array(normals)
        
        weights = np.array([1 / p['residual'] for p in planes] + [1 / centre_plane['residual']])
        normal = np.sum(normals * weights, axis=0) / np.sum(weights)
        normal = normal / np.linalg.norm(normal)

        # up = np.dot(normal, np.array([0, 0, 1]))
        # right = np.dot(normal, np.array([0, 1, 0]))
        # forward = np.dot(normal, np.array([1, 0, 0]))
        roll = 0
        pitch = asin(-normal[1])
        yaw = atan2(normal[0], normal[2])


        return self._radToDeg(np.array([pitch, yaw, roll]))        

    def _trackFrame(self, img, depth):
        self.points = self._getPointsAndClusters(img, depth)

        if len(self.points) > 0:
            planes, centre_plane = self._getPlanes(self.points)
            centre = self._getCentre(self.points)

            rotation = self._getRotation(planes, centre_plane)

            if self.initial_rotation is None:
                self.initial_rotation = rotation.copy()
            
            self.previous_rotation, self.previous_centre = (rotation - self.initial_rotation, self.camera.twoDto3D((int(centre[0]), int(centre[1]))))
            return self.previous_rotation, self.previous_centre
        else:
            return self.previous_rotation, self.previous_centre

    def startTracking(self):
        started = False
        while True:
            # Start timer for fps calculation
            timer = cv2.getTickCount()
            try:
                img = self.camera.read()
                ir = self.camera.getIRimage().copy()
                ir_thresholded = np.asarray(ir[:, :] > 20000, dtype=np.uint8) * 255
                depth = self.camera.getDepthimage().copy()
            except:
                print("Can't read image from camera")
                break

            if started:
                rotation, centre = self._trackFrame(ir_thresholded, depth)
                self.sendData(rotation, centre)
            
            # NEED TO WORK OUT HOW TO TRANSFROM FROM IR TO COLOUR FOR POINT CLOUD !!!!!!!!!!!!!!!!      

            ir_colour = np.array([ir_thresholded, ir_thresholded, ir_thresholded], dtype=np.uint8).transpose(1,2,0)
            ir_colour = ir_colour.copy()
            # ir_colour = self.camera.capture.transformed_color.copy()
            
            for c in self.points:
                polygon = self._getPolygon(c['points'])
                if len(polygon) > 2:
                    cv2.fillPoly(ir_colour, [polygon], (255,0,0))

                cv2.drawKeypoints(ir_colour, c['points'], ir_colour,   (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                cv2.drawKeypoints(ir_colour, [c['centre']], ir_colour, (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                
            
            # Calcuale fps and display
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            cv2.putText(ir_colour, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.flip(ir_colour, 1)
            cv2.imshow("tracking", ir_colour)
            # cv2.imshow("mask", mask * 255)

            # Press q to quit
            k = cv2.waitKey(1)
            if k & 0xFF == ord('q'):
                break
            elif k & 0xFF == ord('p'):
                fig = plt.figure()
                ax = fig.gca(projection='3d')
                
                x = np.linspace(-10, 10, 100)
                y = np.linspace(-10, 10, 100)

                x, y = np.meshgrid(x, y)
                eq = centre_plane['fit'][0] * x + centre_plane['fit'][1] * y + centre_plane['fit'][2]
                ax.plot_surface(x, y, eq)
                for i in planes:
                    try:
                        eq = i['point_plane']['fit'][0] * x + i['point_plane']['fit'][1] * y + i['point_plane']['fit'][2]
                        ax.plot_surface(x, y, eq)
                    except KeyError:
                        pass

                plt.show()
            elif k == ord('s'):
                started = not started
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
        camera = AzureKinectCamera(transformed = False, voxel_size=0.025)
    
    objectTracker = ReflectorTrackNoICP(sock, camera, ip=args.ip, port=args.port)

    objectTracker.startTracking()
    
    cv2.destroyAllWindows()