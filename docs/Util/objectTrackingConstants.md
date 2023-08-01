# Object Tracking Constants

## SiamMask Constants

- `CONFIG`: Path to the SiamMask configuration file.
- `PRE_TRAINED_MODEL_SiamMask`: Path to the pre-trained SiamMask model.

## Re3 Constants

- `PRE_TRAINED_MODEL_Re3`: Path to the pre-trained Re3 model.

## General Object Tracking Constants

- `TRACKER`: The selected object tracker (e.g., "SiamMask").
- `VOXEL_SIZE`: The voxel size for object tracking.
- `COLOUR`: A boolean flag indicating color usage in tracking.
- `NO_ITERATIONS`: Number of iterations for tracking.
- `FAKE_CAMERA`: A boolean flag indicating whether a fake camera is used.

# Calibration Constants

- `AZURE_CALIBRATION`: Path to the Azure Kinect camera calibration file.
- `OPENCV_CALIBRATION`: Path to the OpenCV camera calibration file.
- ... (other calibration-related constants)

# Aruco Constants

- `ARUCO_TYPE`: The type of Aruco marker used.
- `ARUCO_DICT`: Dictionary of Aruco marker types and their corresponding IDs.
- `OFFCET_DICT`: Dictionary of Aruco marker IDs and their associated offset data.
- `ARUCO_ROTATION_OFFCET`: Quaternion representing the Aruco marker rotation offset.

# Fake Camera Constants

- `IMAGE_FILE_LOCATIONS`: Path to the location of benchmark images for the fake camera.

# Benchmark Constants

- `BENCHMARK_FILE`: Path to the benchmark result file.

