# AzureKinect.py

`AzureKinectCamera` is a Python class that represents an Azure Kinect Camera used for capturing images and generating point clouds.

## Usage

```python
from Camera.AzureKinect import AzureKinectCamera

# Create an instance of AzureKinectCamera
camera = AzureKinectCamera()

# Capture an image
image = camera.read()

# Get the depth image
depth_image = camera.getDepthimage()

# Get the infrared image
ir_image = camera.getIRimage()

# Generate a point cloud
point_cloud = camera.getPointCloud(bbox)

# Stop the camera when done
camera.stop()
```

## Class Attributes

- `k4a` (PyK4A): The PyK4A instance for interfacing with the Azure Kinect camera.
- `pcd` (o3d.geometry.PointCloud): An Open3D PointCloud object to store point cloud data.
- `voxel_size` (float): Voxel size used for downsampling the point cloud.
- `min_standard_deviation` (float): Minimum standard deviation used for point cloud thresholding.
- `transformed` (bool): Flag indicating whether to use transformed depth point cloud or not.
- `point_cloud_threshold` (int): Minimum number of points required to generate a point cloud.

## Methods

- `__init__(self, voxel_size=0.05, min_standard_deviation=0.2, point_cloud_threshold=1000, transformed=True)`: Initialize the AzureKinectCamera object.
- `read(self)`: Capture an image from the Azure Kinect camera.
- `getDepthimage(self)`: Get the depth image from the Azure Kinect camera.
- `getIRimage(self)`: Get the infrared image from the Azure Kinect camera.
- `getPointCloud(self, bbox, mask=np.array([]), colour=False)`: Generate a point cloud from the captured image.
- `_getNoColourCloud(self, bbox, mask)`: Generate a point cloud without color information.
- `_getColourCloud(self, bbox, mask)`: Generate a point cloud with color information.
- `stop(self)`: Stop the Azure Kinect camera.
- `get_calibration(self)`: Get the camera matrix and distortion coefficients for color images.
- `twoDto3D(self, coordinate)`: Convert 2D pixel coordinate to 3D point in camera space.
- `getPointCloudCentre(self, pc)`: Get the center of the point cloud in camera space.

## Note

- The `Camera` class is inherited to provide a common interface for different camera types.
- This class uses the `pyk4a` and `Open3D` libraries for interacting with the Azure Kinect camera and processing point clouds.
