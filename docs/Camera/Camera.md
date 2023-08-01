# Camera.py

The `Camera` interface is an abstract class that defines the common methods and attributes for interacting with different camera types in object tracking applications.

## Methods

- `read(self)`: Output a new image from the camera.
- `getDepthimage(self)`: Output the Depth image from the camera.
- `getIRimage(self)`: Output the IR image from the camera.
- `getPointCloud(self, bbox, mask=np.array([]))`: Return an Open3D PointCloud of just the tracked object.
- `stop(self)`: Stop the camera.
- `get_calibration(self)`: Return the intrinsic calibration matrix of the camera.
- `twoDto3D(self, coordinate)`: Converts a 2D coordinate to a 3D one 
- `getPointCloudCentre(self, pc)`: Return the center of the point cloud in meters.

## Note

- This interface serves as a blueprint for implementing different camera types (e.g., Azure Kinect, Webcams) in object tracking applications.
- The methods marked with `@abstractmethod` must be implemented in the derived camera classes for specific camera functionalities.
- Specific camera implementations may require additional libraries, such as `Open3D` for generating and processing point clouds.
