# examples/Python/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
import open3d as o3d
import time


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


if __name__ == "__main__":

    print("1. Load two point clouds and show initial pose")
    source = o3d.io.read_point_cloud("./images/pc1.ply")
    target = o3d.io.read_point_cloud("./images/pc2.ply")

    # source.paint_uniform_color([1, 0.706, 0])
    # target.paint_uniform_color([0, 0.706, 1])

    # draw initial alignment
    current_transformation = np.identity(4)
    draw_registration_result_original_color(source, target,
                                            current_transformation)

    # Croping
    # print("3. Crop point cloud")
    vol = o3d.visualization.read_selection_polygon_volume("./icp/bounding.json")
    source_crop = vol.crop_point_cloud(source)

    # o3d.visualization.draw_geometries([source_crop])