# examples/Python/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
import open3d as o3d
import time


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])

def loadPCD(num, path, colour = np.array([1, 0, 0])):
    pcds = []
    for i in range(0,num):
        print(path+str(i)+".ply")
        pcd = o3d.io.read_point_cloud(path+str(i)+".ply")
        # pcd.paint_uniform_color(np.random.rand(3,1))
        pcds.append(pcd)
        o3d.visualization.draw_geometries([pcd])
    print(len(pcds))
    return pcds



if __name__ == "__main__":

    print("1. Load two point clouds and show initial pose")
    # pcds = loadPCD(12, "images/pc", np.array([1, 0, 0]))
    # originals = loadPCD(10, "images/original", np.array([0, 0, 1]))
    originalPcd = o3d.io.read_point_cloud("images/original.ply")
    # originalPcd.paint_uniform_color([0,0,1])

    # pcd = o3d.io.read_point_cloud("/images/pcBroken.ply")
    # pcd.paint_uniform_color([1,0,0])

    # source_down = source.voxel_down_sample(0.04)
    # target_down = source2.voxel_down_sample(0.04)

    o3d.visualization.draw_geometries([originalPcd])
    # o3d.visualization.draw_geometries(pcds + originals)

    # source_down.estimate_normals(
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=0.04 * 2, max_nn=30))
    # target_down.estimate_normals(
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=0.04 * 2, max_nn=30))
    
    # result_icp = o3d.pipelines.registration.registration_icp(
    #     source_down, target_down, 1.0, np.identity(4),
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane(o3d.pipelines.registration.TukeyLoss(k=0.5)),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
    #                                             relative_rmse=1e-6,
    #                                             max_iteration=100))
    # print(result_icp.transformation)
    # draw_registration_result_original_color(source, target_down, result_icp.transformation)
