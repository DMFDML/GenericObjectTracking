# ReflectorTrack Class

The `ReflectorTrack` class implements an object tracker for retro-reflective objects using the Azure Kinect camera. It provides methods to track and visualize the detected retro-reflective objects in real-time.

## Usage

```python
from ObjectTrackers.reflectorTrack import ReflectorTrack
from Camera.AzureKinectCamera import AzureKinectCamera
import socket

# Set up a UDP socket for sending data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create an instance of the AzureKinectCamera
camera = AzureKinectCamera(voxel_size=0.02)

# Create an instance of the ReflectorTrack object tracker
reflector_tracker = ReflectorTrack(sock, camera, ip=UDP_IP, port=UDP_PORT, voxel_size=VOXEL_SIZE, colour=COLOUR, no_iterations=NO_ITERATIONS)

# Start the object tracking process
reflector_tracker.startTracking()

cv2.destroyAllWindows()
```

## Description

The `ReflectorTrack` class is an object tracker designed for detecting and tracking retro-reflective objects using the Azure Kinect camera. It uses blob detection and alpha shape algorithms to detect the retro-reflective points and estimate their 3D position and orientation in real-time.


## Methods

- `_getPolygon(self, ir)`: Detects and returns the points and polygon of the retro-reflective object from the infrared image.
- `_addToOriginalPointCloud(self, pcd, transform)`: Adds a new point cloud to the reference point cloud and refines it using ICP.
- `_getTranslationRotationMatrix(self, mask, bbox)`: Estimates the translation and rotation matrix of the detected object.
- `_drawPolygon(self, img)`: Draws the detected polygon and bounding box on the image.
- `trackFrame(self, img)`: Tracks and estimates the rotation and translation of the detected object in a single frame.
- `startTracking(self)`: Starts the real-time object tracking process.

## Note

- The `ReflectorTrack` class is designed for retro-reflective objects and uses the Azure Kinect camera for detection and tracking.
- The `trackFrame` method detects and tracks the retro-reflective object in a single frame and returns the estimated rotation and translation.
- The `startTracking` method starts the real-time object tracking process using the `trackFrame` method.
