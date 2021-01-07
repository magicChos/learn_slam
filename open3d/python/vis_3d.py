#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :vis_3d.py
@brief       :连续点云可视化
@time        :2021/01/07 16:52:04
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


import open3d as o3d
import numpy as np
import cv2
import yaml
import argparse
from glob import glob
from tqdm import tqdm
import os


def create_pointcloud_from_depth_and_rgb(depth_name, color_name, cam_matrix, camera_2_base=None):
    '''
    brief        : use rgb image and depth image generate point cloud
    cam_matrix   : camera intrinsic matrix
    camera_2_base: camera to base-link extrinsic matrix
    '''
    color_img = o3d.io.read_image(color_name)
    depth_img = o3d.io.read_image(depth_name)

    height, width = np.asarray(depth_img).shape
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, depth_img)

    # 内参赋值
    fx, fy, cx, cy = cam_matrix[0, 0], cam_matrix[1,
                                                  1], cam_matrix[0, 2], cam_matrix[1, 2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic)

    # Flip it, otherwise the pointcloud will be upside down
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    if camera_2_base != None:
        pcd.transform(camera_2_base)

    return pcd

    # save_point_cloud_name = depth_name.replace("png", "pcd")
    # o3d.io.write_point_cloud(save_point_cloud_name, pcd)

    # # 显示
    # o3d.visualization.draw_geometries([pcd])


def write_pcd(pcd, save_pcd_name):
    '''
    保存点云数据
    '''
    o3d.io.write_point_cloud(save_pcd_name, pcd)


def read_yaml(yaml_file):
    with open(yaml_file, "r") as f:
        data = yaml.load(f)
        return data


def read_yaml_opencv(yml_file):
    fs = cv2.FileStorage(yml_file, cv2.FILE_STORAGE_READ)
    fn = fs.getNode("camera_matrix")
    return fn.mat()


def parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--color', help='color image dir',
                        default="/home/han/data/project/Yolo-Fastest/2021-01-07-tof_data/color")
    parser.add_argument('-d', '--depth', help='depth image dir',
                        default="/home/han/data/project/Yolo-Fastest/2021-01-07-tof_data/depth")
    parser.add_argument("-cf", "--config_file", default="/home/han/data/project/Yolo-Fastest/cfg/yolo-fastest-xl.cfg",
                        help="path to config file")
    parser.add_argument("-df", "--data_file", default="/home/han/data/project/Yolo-Fastest/cfg/voc.data",
                        help="path to data file")
    parser.add_argument("-t", "--thresh", type=float, default=.6,
                        help="remove detections with lower confidence")
    parser.add_argument("-w", "--weight", help="weight file",
                        default="backup/yolo-fastest-xl_last.weights")
    parser.add_argument("-i", "--intrisic", help='camera intrinsic matrix',
                        default="/home/han/data/project/Yolo-Fastest/2021-01-07-tof_data/camera_param.yaml")
    parser.add_argument("-e", "--extrinsic", help="camera extrinsic matrix",
                        default="/home/han/data/project/Yolo-Fastest/2021-01-07-tof_data/calibration.yaml")

    return parser.parse_args()


def main():
    args = parser_args()
    camera_matrix = read_yaml_opencv(args.intrisic)
    extrinsic_matrix = read_yaml(args.extrinsic)
    camera_to_base_matrix = extrinsic_matrix['camera_to_base']


    print("camera_to_base_matrix: ", camera_to_base_matrix)
    print("camera_matrix: ", camera_matrix)

    depth_lst = glob(args.depth + "/*.png")
    color_lst = glob(args.color + "/*.png")
    depth_lst.sort()
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 10
    opt.show_coordinate_frame = True
    
    # pcd = o3d.io.read_point_cloud("/home/han/data/project/Yolo-Fastest/2021-01-07-tof_data/depth/00000_2021-01-07_16-04-33_331.pcd")
    first_depth_name = depth_lst[0]
    first_color_name = first_depth_name.replace('depth', 'color')
    
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    pcd = create_pointcloud_from_depth_and_rgb(first_depth_name, first_color_name , camera_matrix , camera_to_base_matrix)

    geometry = o3d.geometry.PointCloud()
    geometry.points = pcd.points
    vis.add_geometry(geometry)
    vis.add_geometry(axis_pcd)

    
    for depth_name in tqdm(depth_lst[1:]):
        color_name = depth_name.replace('depth', 'color')
        if not os.path.exists(color_name):
            continue
        
        
        tmp = create_pointcloud_from_depth_and_rgb(depth_name, color_name , camera_matrix , camera_to_base_matrix)
        geometry.points = tmp.points

        
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        
        cv2.waitKey(100)
        
        # vis.clear_geometries()
        
        # o3d.visualization.draw_geometries([pcd])
    
    vis.run()    
    vis.destroy_window()




if __name__ == '__main__':
    main()
