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
import argparse


# Outputs the point cloud as a json file
def saveImages(captures, file_name):
    for no, capture in enumerate(captures):
        pc_transform = capture.transformed_depth_point_cloud
        pc_no_transform = capture.depth_point_cloud
        colour = capture.color[:,:, :3]
        depth = capture.depth
        ir = capture.ir

        imageio.imwrite(f"{file_name}/pointcloud_transform/pc" + str(no) + ".tif", pc_transform)
        imageio.imwrite(f"{file_name}/pointcloud_no_transform/pc" + str(no) + ".tif", pc_no_transform)
        imageio.imwrite(f"{file_name}/colour/colour" + str(no) + ".tif", colour)
        imageio.imwrite(f"{file_name}/ir/ir" + str(no) + ".tif", ir)
        imageio.imwrite(f"{file_name}/depth/depth" + str(no) + ".tif", depth)
        print("saved " + str(no))

def main(file_name, camera):

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
            camera_fps=pyk4a.FPS.FPS_30
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

        if camera == 'colour' and np.any(capture.color):
            img = capture.color.copy()
            img = cv2.circle(img, (img.shape[1]//2, img.shape[0]//2), 10, (0,0,255), -1)
            img = cv2.line(img, (img.shape[1]//2, img.shape[0]//2), (img.shape[1]//2+500, img.shape[0]//2), (0,0,255), 2)
            img = cv2.line(img, (img.shape[1]//2, img.shape[0]//2), (img.shape[1]//2, img.shape[0]//2+500), (255,0,0), 2)
            cv2.imshow("capture", img)
        elif camera == 'depth' and np.any(capture.transformed_color):
            img = np.asarray(capture.ir.copy()[:, :] > 20000, dtype=np.uint8) * 255
            img = np.array([img, img, img], dtype=np.uint8).transpose(1,2,0).copy()
            img = cv2.circle(img, (img.shape[1]//2, img.shape[0]//2), 10, (0,0,255), -1)
            img = cv2.line(img, (img.shape[1]//2, img.shape[0]//2), (img.shape[1]//2+500, img.shape[0]//2), (0,0,255), 2)
            img = cv2.line(img, (img.shape[1]//2, img.shape[0]//2), (img.shape[1]//2, img.shape[0]//2+500), (255,0,0), 2)
            cv2.imshow("capture", img)
        # If S is pressed then save the images

        k = cv2.waitKey(1)
        if k == ord('s'):
            break
        elif k == ord('q'):
            exit()
        
    captures = []
    frame_rate = 1/30
    timeframe = []

    for i in range(180):
        start = time.time()
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth):

            captures.append(capture)
            # cv2.imshow("capture", capture.color)
        time.sleep(frame_rate - (time.time() - start))
        
    saveImages(captures, file_name)

    k4a.stop()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Save Images')
    parser.add_argument('--file', dest='file', default=IMAGE_FILE_LOCATIONS, type=str, 
                    help='The location of the files to be stored')
    parser.add_argument('--camera', dest='camera', default="colour", type=str, help='The type of camera being used')
    args = parser.parse_args()
    main(args.file, args.camera)