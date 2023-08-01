# benchmark.py

This script is used for benchmarking the capabilities of each tracker. The script is designed to be used with command-line arguments to specify the tracker type, camera settings, and other parameters.

## Script Overview

The script performs the following tasks:

1. Initializes the chosen object tracker and the camera based on the specified options.
2. Captures images from the camera and tracks the object using the selected object tracker.
3. Simulates Lazy Susan rotation OR uses vive tracking data and calculates rotation and translation errors based on the ground truth values.
4. Records benchmark data (frame number, processing time, rotation, rotation_z, and translation) into a CSV file for performance evaluation.

## Script Usage

To use the script, follow these steps:

1. Run the script using Python with the appropriate command-line arguments.

```bash
python script_name.py --box [x,y,w,h] --tracker TrackerType --file VideoFilePath --voxel_size VoxelSize --colour ColourICP --no_iterations NumIterations --aruco_type ArucoType --marker_shape MarkerSize --vive UseViveTracker
```

Here:

- `[x,y,w,h]` represents the bounding box coordinates [x, y, width, height].
- `TrackerType` specifies the type of tracker to use. Choose from "GenericObjectTracker", "ReflectorTracker", "ReflectorTrackerNoICP", or "ArucoTracker".
- `VideoFilePath` is the path to the video or image sequence file to use as input.
- `VoxelSize` is the size of voxels to be used for ICP (applicable for GenericObjectTracker).
- `ColourICP` should be either "True" or "False" to indicate whether color ICP is used or not (applicable for GenericObjectTracker).
- `NumIterations` is the number of iterations to perform ICP (applicable for GenericObjectTracker).
- `ArucoType` is the type of Aruco markers used (applicable for ArucoTracker).
- `MarkerSize` is the size of the Aruco markers (applicable for ArucoTracker).
- `UseViveTracker` should be "True" or "False" to indicate whether the Vive tracker is used or not.

## Script Description

The main loop captures images from the camera and performs object tracking using the selected tracker. It also calculates the processing time for each frame. If Vive tracking is enabled, the script reads the Vive positional data from a CSV file and uses this as ground truth otherwise it uses the Lazy Susan function to calculate the ground truth on a spining disk.

The benchmark data, including frame number, processing time, rotation, rotation_z, and translation, is recorded into a CSV file for performance evaluation.
