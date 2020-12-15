#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :draw_3dbox.py
@brief       :绘制3dbox
@time        :2020/12/10 15:45:43
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


import cv2
import numpy as np
import argparse
import os
import open3d as o3d
from glob import glob
from tqdm import tqdm


def vis_depth_img(depth_img):
    '''
    可视化深度图
    '''
    cv2.imshow("depth", depth_img * 255)
    cv2.waitKey(0)


def vis_depth_and_color(depth_img, color_img):
    '''
    同时可视化深度图和rgb图
    '''
    height, width = color_img.shape[:2]
    full_img = np.zeros((height, 2 * width, 3), dtype=np.uint8)

    depth_img = cv2.applyColorMap(cv2.convertScaleAbs(
        depth_img, alpha=15), cv2.COLORMAP_JET)
    full_img[:, :width] = color_img
    full_img[:, width:] = depth_img

    cv2.imshow("depth & color", full_img)


def convert_open3d_2_numpy(open3d_pointcloud):
    '''
    将open3d的pointcloud转为Numpy
    '''
    return np.asarray(open3d_pointcloud.points)


def convert_numpy_2_open3d(numpy_points):
    '''
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(numpy_points)
    return pcd


def vis_pointcloud(points_3d, save_point_cloud=None):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_3d)

    max_bound = point_cloud.get_max_bound()
    min_bound = point_cloud.get_min_bound()

    print("max_bound: ", max_bound)
    print("min bound: ", min_bound)

    print("点云数量： ", point_cloud)
    alignbbox = point_cloud.get_axis_aligned_bounding_box()
    alignbbox.color = (1, 0, 0)
    orientdbbox = point_cloud.get_oriented_bounding_box()
    orientdbbox.color = (0, 1, 0)

    # o3d.visualization.draw_geometries(
    #     [point_cloud, alignbbox, orientdbbox], width=600, height=600)

    if save_point_cloud != None:
        o3d.io.write_point_cloud(save_point_cloud, point_cloud)


camera_matrix = np.array([[705.13641, 0, 330.80023],
                          [0, 704.5658, 240.67661],
                          [0, 0, 1]])

bndbox = [275, 166, 464, 251]


def draw_3d_box2(img, bvs, stx=0, sty=0):
    col1 = (0, 0, 255)
    col2 = (255, 0, 0)

    cls = [col1, col1, col1, col1, col2, col2,
           col1, col1, col1, col1, col1, col2]

    vs = []
    for i, var in enumerate(bvs):
        vs.append((int(stx+var[0]), int(sty+var[1])))

        cv2.putText(img, '%d' %
                    i, vs[i], cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 225), 2)

    cv2.line(img, vs[0], vs[1], cls[0], 2, 8, 0)
    cv2.line(img, vs[1], vs[2], cls[1], 2, 8, 0)
    cv2.line(img, vs[2], vs[3], cls[2], 2, 8, 0)
    cv2.line(img, vs[3], vs[0], cls[3], 2, 8, 0)

    cv2.line(img, vs[4], vs[5], cls[4], 2, 8, 0)
    cv2.line(img, vs[5], vs[6], cls[5], 2, 8, 0)
    cv2.line(img, vs[6], vs[7], cls[6], 2, 8, 0)
    cv2.line(img, vs[7], vs[4], cls[7], 2, 8, 0)

    cv2.line(img, vs[0], vs[4], cls[8], 2, 8, 0)
    cv2.line(img, vs[3], vs[7], cls[9], 2, 8, 0)
    cv2.line(img, vs[2], vs[6], cls[10], 2, 8, 0)
    cv2.line(img, vs[1], vs[5], cls[11], 2, 8, 0)


def get_roi_point_cloud(depth_img, cam_matrix, object_roi, depth_scale=1000):
    '''
    根据检测的box，取对应的点云数据
    '''
    cam_fx = cam_matrix[0][0]
    cam_fy = cam_matrix[1][1]
    cam_cx = cam_matrix[0][2]
    cam_cy = cam_matrix[1][2]
    factor = depth_scale

    m, n = depth_img.shape
    roi_point_cloud = []
    for v in range(m):
        for u in range(n):
            if object_roi[0] < u < object_roi[2] and object_roi[1] < v < object_roi[3]:
                if depth_img[v, u] <= 500 or depth_img[v, u] >= 2000:
                    continue
                depth = depth_img[v, u]
                p_z = depth / factor
                p_x = (u - cam_cx) * p_z / cam_fx
                p_y = (v - cam_cy) * p_z / cam_fy

                roi_point_cloud.append([p_x, p_y, p_z])

    roi_point_cloud = np.array(roi_point_cloud)
    return roi_point_cloud


def depth2pc(depth_img, cam_matrix, depth_scale=1000):
    """
    深度图转点云数据
    图像坐标系 -> 世界坐标系 
    :param depth_img: 深度图
    :return: 点云数据 N*3
    """

    cam_fx = cam_matrix[0][0]
    cam_fy = cam_matrix[1][1]
    cam_cx = cam_matrix[0][2]
    cam_cy = cam_matrix[1][2]
    factor = depth_scale

    # 逐点处理，此过程可以使用numpy优化
    m, n = depth_img.shape
    point_cloud = []
    for v in range(m):
        for u in range(n):
            if depth_img[v, u] <= 500 or depth_img[v, u] >= 2000:
                continue
            depth = depth_img[v, u]
            p_z = depth / factor
            p_x = (u - cam_cx) * p_z / cam_fx
            p_y = (v - cam_cy) * p_z / cam_fy
            point_cloud.append([p_x, p_y, p_z])

    point_cloud = np.array(point_cloud)

    return point_cloud


def depth2xyz(depth_map, depth_cam_matrix, flatten=False, depth_scale=1000):
    fx, fy = depth_cam_matrix[0, 0], depth_cam_matrix[1, 1]
    cx, cy = depth_cam_matrix[0, 2], depth_cam_matrix[1, 2]
    h, w = np.mgrid[0:depth_map.shape[0], 0:depth_map.shape[1]]
    z = depth_map/depth_scale
    x = (w-cx)*z/fx
    y = (h-cy)*z/fy
    xyz = np.dstack((x, y, z)) if flatten == False else np.dstack(
        (x, y, z)).reshape(-1, 3)

    return xyz


def project_Camera_3d_to_image(camera_3d_pt, calibration_matrix):
    '''
    将摄像机坐标系下的三维点投影到图像上
    '''
    camera_3d_pt[:, :2] /= camera_3d_pt[:, 2]
    a = camera_3d_pt[:, 0]
    b = camera_3d_pt[:, 1]

    # K = np.array(calibration_info['K'])
    # C = np.array(calibration_info['C'])
    # F = np.array(calibration_info['F'])

    F = []
    F.append(calibration_matrix[0][0])
    F.append(calibration_matrix[1][1])
    C = []
    C.append(calibration_matrix[0][2])
    C.append(calibration_matrix[1][2])

    # r = np.sqrt(np.power(a, 2) + np.power(b, 2))
    # ad = np.multiply(a, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4)) + np.multiply(
    #     K[4], np.power(r, 6)))) + 2 * K[2] * np.multiply(a, b) + K[3] * (np.power(r, 2) + 2 * np.power(a, 2))
    # bd = np.multiply(b, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4)) + np.multiply(
    #     K[4], np.power(r, 6)))) + K[2] * (np.power(r, 2) + 2 * np.power(b, 2)) + 2 * K[3] * np.multiply(a, b)

    pixel_x = (F[0] * a + C[0])
    pixel_y = (F[1] * b + C[1])
    pixel_list = np.hstack((pixel_x, pixel_y))
    return pixel_list


def parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--color', help='color image dir', required=True)
    parser.add_argument('-d', '--depth', help='depth image dir', required=True)
    return parser.parse_args()


def process_single_image(depth_name, color_name, cam_matrix):
    create_pointcloud_from_depth_and_rgb(depth_name, color_name, cam_matrix)


def main():
    args = parser_args()

    depth_lst = glob(args.depth + "/*.png")
    color_lst = glob(args.color + "/*.png")

    
    for depth_name in tqdm(depth_lst):
        color_name = depth_name.replace('depth', 'color')
        if not os.path.exists(color_name):
            continue

        process_single_image(depth_name, color_name, camera_matrix)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    # color_name = "tof_data/color/00000_2020-12-10_14-31-03_172.png"
    # depth_name = "tof_data/depth/00000_2020-12-10_14-31-03_172.png"

    # color_img = cv2.imread(color_name)
    # depth_img = cv2.imread(depth_name, -1)

    # # point_xyz = depth2pc(depth_img, camera_matrix, True)
    # # vis_pointcloud(point_xyz)

    # point_xyz = get_roi_point_cloud(
    #     depth_img, camera_matrix, bndbox, depth_scale=1000)

    # xmax, ymax, zmax = point_xyz.max(0)
    # xmin, ymin, zmin = point_xyz.min(0)

    # print(
    #     f"xmin: {xmin} , ymin: {ymin} , zmin: {zmin} , xmax: {xmax} , ymax: {ymax} , zmax: {zmax}")

    # bboxes = []
    # bboxes.append([xmin, ymin, zmin])
    # bboxes.append([xmax, ymin, zmin])
    # bboxes.append([xmax, ymax, zmin])
    # bboxes.append([xmin, ymax, zmin])

    # bboxes.append([xmin, ymin, zmax])
    # bboxes.append([xmax, ymin, zmax])
    # bboxes.append([xmax, ymax, zmax])
    # bboxes.append([xmin, ymax, zmax])

    # bboxes = np.matrix(bboxes, dtype=np.float32)
    # project_pts = project_Camera_3d_to_image(bboxes, camera_matrix).tolist()

    # draw_3d_box2(color_img, project_pts)

    # vis_pointcloud(point_xyz, "shoes.ply")

    # depth_img_large = depth_img * 255

    # cv2.imshow("color", color_img)
    # cv2.imshow("depth", depth_img_large)
    # cv2.waitKey(0)


def create_pointcloud_from_depth(depth_name, cam_matrix):
    '''
    利用深度图获取点云
    '''
    depth_img = o3d.io.read_image(depth_name)
    height, width = np.asarray(depth_img).shape
    # 内参赋值
    fx, fy, cx, cy = cam_matrix[0][0], cam_matrix[1][1], cam_matrix[0][2], cam_matrix[1][2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_img, intrinsic)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])


def create_pointcloud_from_depth_and_rgb(depth_name, color_name, cam_matrix):
    '''
    利用深度图和rgb图获取点云
    '''
    color_img = o3d.io.read_image(color_name)
    depth_img = o3d.io.read_image(depth_name)

    depth_img_numpy = np.asarray(depth_img)
    color_img_numpy = np.asarray(color_img)

    vis_depth_and_color(depth_img_numpy, color_img_numpy)

    height, width = np.asarray(depth_img).shape
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, depth_img)

    # 内参赋值
    fx, fy, cx, cy = camera_matrix[0][0], camera_matrix[1][1], camera_matrix[0][2], camera_matrix[1][2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic)

    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # 显示
    # o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    main()

    # depth_name = "tof_data/depth/00000_2020-12-10_14-31-03_172.png"
    # color_name = "tof_data/color/00000_2020-12-10_14-31-03_172.png"
    # create_pointcloud_from_depth(depth_name , camera_matrix)
    # create_pointcloud_from_depth_and_rgb(depth_name , color_name , camera_matrix)
