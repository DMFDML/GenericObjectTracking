# OpenCVCamera.py

The `OpenCVCamera` class is an implementation of the abstract `Camera` class that provides access to an OpenCV camera. This class is designed to capture real-time video feed and perform camera calibration using checkerboard calibration images.

## Usage

```python
from Camera.OpenCVCamera import OpenCVCamera

# Set the path to calibration images
calibration_images = 'images/calibration/calibrate*.png'

# Create an instance of the OpenCVCamera class
opencv_camera = OpenCVCamera(calibration_images=calibration_images)

try:
    # Read an image from the camera
    image = opencv_camera.read()
    cv2.imshow('Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Stop the camera
    opencv_camera.stop()

except Exception as e:
    print("An error occurred:", str(e))
```

## Description

The `OpenCVCamera` class provides access to an OpenCV camera. It allows users to capture real-time video feed, read images from the camera, and perform camera calibration using checkerboard calibration images. The intrinsic calibration matrix and distortion coefficients are saved in a JSON file for future use.

## Methods

- `read(self)`: Output a new image from the camera.
- `getDepthimage(self)`: Not implemented (raises `NotImplementedError`).
- `getIRimage(self)`: Not implemented (raises `NotImplementedError`).
- `getPointCloud(self, bbox, mask=np.array([]))`: Not implemented (raises `NotImplementedError`).
- `stop(self)`: Stop the camera.
- `calibrate_camera(self, calibration_images, checkerBoard=CHECKERBOARD)`: Calibrate the camera using checkerboard calibration images.
- `get_calibration(self)`: Return the intrinsic calibration matrix and distortion coefficients.
- `twoDto3D(self, input2D)`: Not implemented (raises `NotImplementedError`).
- `getPointCloudCentre(self, pc)`: Not implemented (raises `NotImplementedError`).

## Note

- The `OpenCVCamera` class requires access to a connected camera to capture real-time video feed.
- Camera calibration is performed using checkerboard calibration images to obtain the intrinsic calibration matrix and distortion coefficients.
- The calibration data is saved in a JSON file named `calibration_opencv.json` in the same directory as the script.

---

Save the content above into a file named `OpenCVCameraClass.md` in the root directory of your project to create a Markdown readme file for the `OpenCVCamera` class, including an example of how to use the class.