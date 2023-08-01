# reflectorTrackNoICP.py

The `ReflectorTrackNoICP` class implements an object tracker for retro-reflective objects using the Azure Kinect camera. It provides methods to track and visualize the detected retro-reflective objects in real-time without using the Iterative Closest Point (ICP) algorithm.

## Usage

```python
from ObjectTrackers.reflectorTrackNoICP import ReflectorTrackNoICP
from Camera.AzureKinectCamera import AzureKinectCamera
import socket

# Set up a UDP socket for sending data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create an instance of the AzureKinectCamera
camera = AzureKinectCamera(voxel_size=0.025)

# Create an instance of the ReflectorTrackNoICP object tracker
reflector_tracker_no_icp = ReflectorTrackNoICP(sock, camera, ip=UDP_IP, port=UDP_PORT)

# Start the object tracking process
reflector_tracker_no_icp.startTracking()

cv2.destroyAllWindows()
```

## Description

The `ReflectorTrackNoICP` class is an object tracker designed for detecting and tracking retro-reflective objects using the Azure Kinect camera. It uses blob detection and alpha shape algorithms to detect the retro-reflective points and estimates their 3D position and orientation in real-time without using the Iterative Closest Point (ICP) algorithm.

## Class Attributes

- `points`: A list that stores the detected clusters and their points.
- `previous_rotation`: A numpy array representing the previous rotation of the detected object.
- `previous_centre`: A numpy array representing the previous center position of the detected object.

## Methods

- `_getPolygon(self, points)`: Detects and returns the polygon of the retro-reflective object from the detected points.
- `_getPointsAndClusters(self, ir, depth)`: Detects and returns the clusters and points of the retro-reflective objects from the infrared and depth images.
- `_getPlanes(self, points)`: Calculates and returns the planes fitted to the detected clusters and all cluster centers.
- `_getCentre(self, points)`: Calculates and returns the average center of all detected points.
- `_getRotation(self, planes, centre_plane)`: Calculates and returns the average Euler angles of the normal for all detected planes.
- `_drawPoints(self, img, points)`: Draws the detected clusters and their points on the image.
- `trackFrame(self, img, depth)`: Tracks and estimates the rotation and center of the detected object in a single frame.
- `startTracking(self)`: Starts the real-time object tracking process.

## Note

- The `ReflectorTrackNoICP` class is designed for retro-reflective objects and uses the Azure Kinect camera for detection and tracking.
- The `trackFrame` method detects and tracks the retro-reflective objects in a single frame and returns the estimated rotation and center.
- The `startTracking` method starts the real-time object tracking process using the `trackFrame` method.
