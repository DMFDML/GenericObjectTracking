import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration, CalibrationType
import open3d as o3d
import json
import imageio


# Outputs the point cloud as a json file
def outputPCImage(capture, no):
    cv2.imwrite(f"images/calibration/azureCalibrate{no}.png", capture.color)
    print(f"writing image images/calibration/calibrate{no}.png")


def main():

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )
    k4a.start()

    # getters and setters directly get and set on device
    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    pcd = o3d.geometry.PointCloud()

    save = 1
    print(k4a.calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.COLOR), " ", k4a.calibration.get_distortion_coefficients(pyk4a.calibration.CalibrationType.COLOR))

    while True:
        capture = k4a.get_capture()

        if np.any(capture.color) and np.any(capture.depth):

            cv2.imshow("image", capture.color)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            
            outputPCImage(capture, save)
            save+=1


    k4a.stop()


if __name__ == "__main__":
    main()