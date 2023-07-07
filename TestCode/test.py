import cv2
import json
import numpy as np
import imageio
import open3d as o3d

image = imageio.imread("images/pcds/pc0.tif")
print(image.shape)
print(image.max(axis=1), " ", image.min(axis=1))
pc = image.reshape(-1, 3).astype(np.float64)
pc = pc / 255
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc)
o3d.visualization.draw_geometries([pcd])
