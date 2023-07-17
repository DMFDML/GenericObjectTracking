import sys
sys.path.append("./src")
import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration, CalibrationType
import open3d as o3d
import json
import imageio
from Util.objectTrackingConstants import IMAGE_FILE_LOCATIONS
import time


# Outputs the point cloud as a json file
def saveImages(captures):
    for no, capture in enumerate(captures):
        pc_transform = capture.transformed_depth_point_cloud
        pc_no_transform = capture.transformed_depth_point_cloud
        colour = capture.color[:,:, :3]
        depth = capture.depth
        ir = capture.ir

        imageio.imwrite(f"{IMAGE_FILE_LOCATIONS}/pointcloud_transform/pc" + str(no) + ".tif", pc_transform)
        imageio.imwrite(f"{IMAGE_FILE_LOCATIONS}/pointcloud_no_transform/pc" + str(no) + ".tif", pc_no_transform)
        imageio.imwrite(f"{IMAGE_FILE_LOCATIONS}/colour/colour" + str(no) + ".tif", colour)
        imageio.imwrite(f"{IMAGE_FILE_LOCATIONS}/ir/ir" + str(no) + ".tif", ir)
        imageio.imwrite(f"{IMAGE_FILE_LOCATIONS}/depth/depth" + str(no) + ".tif", depth)
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

    while True:
        capture = k4a.get_capture()

        if np.any(capture.color) and np.any(capture.depth):
            cv2.imshow("capture", capture.color)
        # If S is pressed then save the images
        if cv2.waitKey(1) == ord('s'):
            break
        
    captures = []
    frame_rate = 1/30
    timeframe = []

    for i in range(90):
        start = time.time()
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth):

            captures.append(capture)
            # cv2.imshow("capture", capture.color)
        time.sleep(frame_rate - (time.time() - start))
        
    saveImages(captures)

    k4a.stop()


if __name__ == "__main__":
    main()