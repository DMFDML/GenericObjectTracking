# genericObjectTracker.py

The `GenericObjectTracker` class is an implementation of the abstract `ObjectTracker` class that provides a generic object tracking functionality. It can use different 2D object trackers (`SiamMaskSharpTracker` or `Re3Tracker`) to track objects in the camera's field of view and estimate their pose (rotation and translation).

## Usage

```python
from ObjectTrackers.genericObjectTracker import GenericObjectTracker
from Camera.OpenCVCamera import OpenCVCamera
import socket


# Define GenericObjectTracker parameters
ip = "127.0.0.1"
port = 12345
voxel_size = 0.05
colour = True
no_iterations = 150
box = []  # Initial bounding box to track (can be selected interactively)

# Create a socket for sending data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create an instance of the AzureKinectCamera class
camera = AzureKinectCamera(voxel_size=voxel_size)

tracker = SiamMaskSharpTracker()

# Create an instance of the GenericObjectTracker class
objectTracker = GenericObjectTracker(sock, tracker, camera, ip=ip, port=port, voxel_size=voxel_size, colour=colour, no_iterations=no_iterations, box=box)

try:
    # Start tracking the object
    objectTracker.startTracking()

    # Close the OpenCV window when the tracking is stopped
    cv2.destroyAllWindows()

except Exception as e:
    print("An error occurred:", str(e))
```

## Description

The `GenericObjectTracker` class provides a generic object tracking functionality by using a 2D object tracker (`SiamMaskSharpTracker` or `Re3Tracker`) to track objects in the camera's field of view. It estimates the pose (rotation and translation) of the tracked objects and sends the pose data over UDP to a specified IP address and port. The class also performs Iterative Closest Point (ICP) registration on the 3D point clouds generated from the tracked bounding boxes to refine the pose estimation.

## Methods

- `_startTrackingBox(self, box = [])`: Prompts the user to draw a bounding box around the object to be tracked.
- `_getOnlyBox(self, img, box)`: Returns a shrunk image of just the bounding box.
- `_addToOriginalPointCloud(self, pcd, transform)`: Adds a new point cloud to the reference point cloud and refines it using point-to-plane ICP.
- `_getTranslationRotationMatrix(self, bbox)`: Performs ICP or colored ICP between the reference point cloud and the new one to get the translation and rotation matrices.
- `trackFrame(self, img)`: Track the object in a single frame and calculate the pose (rotation and translation).
- `startTracking(self)`: Start tracking the object in real-time using the camera feed.

## Note

- The `GenericObjectTracker` class requires access to a connected camera to capture real-time video feed.
- The class works with different 2D object trackers, and you can specify the type of tracker when creating an instance of the class.
- The class sends the estimated pose data (rotation and translation) of the tracked object to the specified IP address and port using UDP.
