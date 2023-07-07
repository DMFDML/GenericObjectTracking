import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration, CalibrationType
import open3d as o3d
import json
import imageio


# Outputs the point cloud as a json file
def outputPCImage(capture, no):
    pc = capture.transformed_depth_point_cloud
    colour = capture.color[:,:, :3]

    imageio.imwrite("images/pcds/pc" + str(no) + ".tif", pc)
    imageio.imwrite("images/colours/colour" + str(no) + ".jpg", colour)
    print("saved " + str(no))

    



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

    for i in range(100):
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth):

            outputPCImage(capture, i)


    k4a.stop()


if __name__ == "__main__":
    main()