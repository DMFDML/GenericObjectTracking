# saveBenchmarkImages.py

This script is used for capturing images from a camera (color or depth) and saving them along with Vive positional data (if available). The script is designed to be run with command-line arguments to specify the camera type and the output file location.

## Script Overview

The script is designed to perform the following tasks:

1. Capture images from a camera (color or depth) using the Azure Kinect camera (PyK4A library).
2. Optionally, capture Vive positional data using the Triad OpenVR library.
3. Save the captured images and Vive positional data to appropriate folders.

## Script Usage

To use the script, follow these steps:

1. Run the script using Python with the appropriate command-line arguments.

```bash
python script_name.py --file OutputFileLocation --camera CameraType --vive UseViveData
```

Here, replace `OutputFileLocation` with the path to the folder where images and Vive data will be saved, `CameraType` with either "colour" or "depth" to specify the camera type, and `UseViveData` with "True" or "False" to indicate whether Vive data is being used or not.

2. While the camera is active, the script captures images and displays them on the screen. If the 'S' key is pressed, the script saves the images and Vive positional data (if available) to the specified output folder.

If Vive data is being used (indicated by the `--vive True` command-line argument), the script uses the Triad OpenVR library to obtain the Vive positional data for a fixed duration of time (180 frames). It stores the captured images in separate folders based on their type (color, depth, etc.), and saves the Vive positional data in a CSV file named `vive.csv` within the output folder.
