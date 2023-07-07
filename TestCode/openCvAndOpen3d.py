import numpy as np
import copy
import open3d as o3d
import time
import cv2
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)



# while True:
#     timer = cv2.getTickCount()
#     success, img = cap.read()

#     if (not success):
#         break

#     rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(img), o3d.geometry.Image(np.ones((480, 640), dtype=np.float32)))

#     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
#         rgbd_image,
#         o3d.camera.PinholeCameraIntrinsic(
#             o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

#     pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#     # o3d.visualization.draw_geometries([pcd])

#     fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
#     cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#     cv2.imshow("tracking", img)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()

_, img = cap.read()

bbox = cv2.selectROI("tracking", img, False)
small_box = img[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
cv2.imshow("box", small_box)

original_width = 640  # Width of the original image in pixels
original_height = 480  # Height of the original image in pixels

scale_x = small_box.shape[1] / original_width
scale_y = small_box.shape[0] / original_height


intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
    )
fx, fy = intrinsic.get_focal_length()
cx, cy = intrinsic.get_principal_point()



rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(small_box), o3d.geometry.Image(np.ones((small_box.shape[0], small_box.shape[1]), dtype=np.float32)))

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        small_box.shape[1], small_box.shape[0], fx, fy, cx, cy
    ))

pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

# cv2.imshow("tracking", img)
