# ArucoTracker Class

The `ArucoTracker` class is an implementation of the abstract `ObjectTracker` class that tracks Aruco markers using computer vision techniques. It uses OpenCV's Aruco marker detection to estimate the pose (rotation and translation) of Aruco markers in the camera's field of view.

## Usage

```python
from ObjectTrackers.arucoTrack import ArucoTrack
from Camera.OpenCVCamera import OpenCVCamera
import socket



# Define Aruco tracker parameters
aruco_type = "DICT_6X6_250"
ip = "127.0.0.1"
port = 12345
marker_size = 40

# Create a socket for sending data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create an instance of the OpenCVCamera class
camera_id = 0
calibration_images = 'images/calibration/azureCalibrate*.png'
calibration_file = None
camera = OpenCVCamera(camera_id=camera_id, calibration_images=calibration_images, calibration_file=calibration_file)

# Create an instance of the ArucoTracker class
objectTracker = ArucoTracker(sock, camera, aruco_type, ip=ip, port=port, marker_size=marker_size)

try:
    # Start tracking Aruco markers
    objectTracker.startTracking()

    # Close the OpenCV window when the tracking is stopped
    cv2.destroyAllWindows()

except Exception as e:
    print("An error occurred:", str(e))
```

## Description

The `ArucoTracker` class is used to track Aruco markers using computer vision techniques. It estimates the pose (rotation and translation) of Aruco markers in the camera's field of view and sends the pose data over UDP to a specified IP address and port. The class uses OpenCV's Aruco marker detection to identify and track the markers.

## Methods

- `_aruco_display(self, corners, ids, rvecs, tvecs, image, colour=(0, 0, 255))`: Display Aruco markers and their poses on the image.
- `_calculateRotationAndTranslation(self, corners, ids)`: Calculate the rotation, translation, and quaternion values for detected Aruco markers.
- `trackFrame(self, img)`: Track Aruco markers in a single frame and calculate the pose (rotation and translation).
- `startTracking(self)`: Start tracking Aruco markers in real-time using the camera feed.

## Note

- The `ArucoTracker` class requires access to a connected camera to capture real-time video feed.
- The class works with various types of Aruco markers. The type of Aruco marker to use can be specified when creating an instance of the class.
- The class sends the estimated pose data (rotation and translation) of the closest Aruco marker to the specified IP address and port using UDP.
