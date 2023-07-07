import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration, CalibrationType
import open3d as o3d
import json
import imageio

def depth_to_image(depth):
    depth_img = depth / depth.max() * 255
    depth_img = depth_img.astype(np.uint8)
    return depth_img

def draw_bounding_boxs(img, bbox, colour=(0,0,255)):
    x,y,w,h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(img, (x,y), ((x+w), (y+h)), colour, 3, 1)
    
# img = 0
# # Outputs the point cloud as a json file
# def outputPCImage(pc):
#     # print(pc.dtype )
#     # print(pc.max(axis=1), " ", pc.min(axis=1))
#     global img
#     # pc = (pc - pc.min()) / (pc.max() - pc.min())
#     print(pc.max(), " ", pc.min(), " ", pc.dtype)
#     imageio.imwrite("images/pcds/pc" + str(img) + ".tif", pc)
#     img += 1

def kinect_to_open3d(capture, pcd, bbox, mask = np.array([])):

    img = capture.transformed_depth_point_cloud
    # print(img[img.shape[0]//2, img.shape[1]//2])
    pc = capture.transformed_depth_point_cloud[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1, 3).astype(np.float64)
    colour = capture.color[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2], :3].reshape(-1, 3).astype(np.float64)  
    if mask.size != 0:
        mask = mask[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].reshape(-1).astype(np.bool)
        pc[mask]
        colour[mask]
    
    colour = colour /255
    
    pcd.points = o3d.utility.Vector3dVector(pc)
    pcd.colors = o3d.utility.Vector3dVector(colour)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # pcd.points.extend(pc)
    # o3d.visualization.draw_geometries([pcd])
    



def main():

    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            
        )
    )
    k4a.start()

    # getters and setters directly get and set on device
    k4a.whitebalance = 4500
    assert k4a.whitebalance == 4500
    k4a.whitebalance = 4510
    assert k4a.whitebalance == 4510

    pcd = o3d.geometry.PointCloud()
    capture = k4a.get_capture()
    # bbox = cv2.selectROI("k4a", capture.color[:, :, :3], False)

    # print(k4a.calibration.depth_mode)
    
    # mask = np.zeros((capture.color.shape[0], capture.color.shape[1]))
    # mask[bbox[1]+10:bbox[1]+bbox[3]-20, bbox[0]+10:bbox[0]+bbox[2]-20] = 1

    save = 1

    while True:
        capture = k4a.get_capture()
        if np.any(capture.color) and np.any(capture.depth):

            colour = capture.color[:, :, :3].copy()
            depth = depth_to_image(capture.transformed_depth).copy()
            # print(colour.shape, " ", depth.shape, " ", colour.dtype, " ", depth.dtype)
            ir = np.asarray(capture.ir[:, :] > 20000, dtype=np.uint8) * 255
            # transformed_ir  = np.asarray(capture.transformed_ir[:, :] > 15000, dtype=np.uint8) * 255
            # print(capture.transformed_ir.max())

            # draw_bounding_boxs(colour, bbox)
            # draw_bounding_boxs(depth, bbox, colour=(255,255,255))
            # print(capture.depth_point_cloud.shape, " ", capture.transformed_depth_point_cloud.shape)

            # cv2.imshow("k4a", colour)
            # cv2.imshow("depth", depth)
            # cv2.imshow("ir", ir)
            # cv2.imshow("transformed", depth_to_image(capture.depth))
            
            # kinect_to_open3d(capture, pcd, bbox)
            ir = cv2.dilate(ir, np.ones((10,10), np.uint8), iterations=2)
            # closing = cv2.morphologyEx(ir, cv2.MORPH_CLOSE, np.ones((10,10), np.uint8), iterations=3)
            thinned = cv2.ximgproc.thinning(ir, thinningType=cv2.ximgproc.THINNING_GUOHALL)
            

            # circles = cv2.HoughCircles(ir, cv2.HOUGH_GRADIENT, 1.5, 20, param1=60, param2=30, minRadius=0, maxRadius=0)
            # ir_colour = np.array([ir, ir, ir], dtype=np.uint8).transpose(1,2,0)
            # ir_colour = ir_colour.copy()

            # if circles is not None:
            #     for c in circles[0]:
            #         # print("adding", c)
            #         cv2.circle(ir_colour, (int(c[0]), int(c[1])), int(c[2]), (0, 0, 255), 2)

            

            cv2.imshow("ir", thinned)
            # cv2.imshow("transformed", transformed_ir)



        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        elif k & 0xFF == ord('s'):
            print("saving image")
            o3d.io.write_point_cloud("./images/pc" + str(save)+".ply", pcd)
            save += 1
    cv2.destroyAllWindows()
    k4a.stop()


if __name__ == "__main__":
    main()