#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :generate_pointcloud.py
@brief       :利用深度图和rbg图生成点云数据
@time        :2021/01/09 23:35:48
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''

import numpy as np
import open3d as o3d
import argparse
import sys
import cv2
from glob import glob
from tqdm import tqdm
import os

sys.path.append("/home/han/data/project/learn_slam/open3d/python/perceptionMagic")

from carline.draw_car_line import DrawCarLine
from utils.yaml_reader import  read_yaml_cv , read_yaml

root_dir = "/home/han/data/project/yolo-fastest/tof_data"
current_dir = "/home/han/data/project/learn_slam/open3d/python/perceptionMagic"

def vis_pointcloud(points_3d, save_point_cloud=None):
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])

    o3d.visualization.draw_geometries(
        [points_3d , axis_pcd], width=600, height=600)

    if save_point_cloud != None:
        o3d.io.write_point_cloud(save_point_cloud, points_3d)


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
        color_img, depth_img , convert_rgb_to_intensity=False)


    # 内参赋值
    fx, fy, cx, cy = cam_matrix[0, 0], cam_matrix[1,
                                                  1], cam_matrix[0, 2], cam_matrix[1, 2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic)
    
    if camera_2_base != None:
        pcd.transform(camera_2_base)
    pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
  
    
    return pcd

def create_pointcloud_from_depth_and_rgb_cv(depth_img_cv , color_img_cv , cam_matrix , camera_2_base = None):
    '''
    brief        : use rgb image and depth image generate point cloud
    cam_matrix   : camera intrinsic matrix
    camera_2_base: camera to base-link extrinsic matrix
    '''
    # color_img = o3d.io.read_image(color_name)
    # depth_img = o3d.io.read_image(depth_name)
    
    color_img = o3d.geometry.Image(color_img_cv)
    depth_img = o3d.geometry.Image(depth_img_cv)

    height, width = np.asarray(depth_img).shape
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, depth_img , convert_rgb_to_intensity=False)


    # 内参赋值
    fx, fy, cx, cy = cam_matrix[0, 0], cam_matrix[1,
                                                  1], cam_matrix[0, 2], cam_matrix[1, 2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic)
    
    if camera_2_base != None:
        pcd.transform(camera_2_base)
    # pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
  
    
    return pcd


def parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--color', help='color image dir',
                        default=f"{root_dir}/color")
    parser.add_argument('-d', '--depth', help='depth image dir',
                        default=f"{root_dir}/depth")
    parser.add_argument("-cf", "--config_file", default=f"{current_dir}/cfg/yolo-fastest-xl.cfg",
                        help="path to config file")
    parser.add_argument("-df", "--data_file", default=f"{current_dir}/cfg/voc.data",
                        help="path to data file")
    parser.add_argument("-t", "--thresh", type=float, default=.6,
                        help="remove detections with lower confidence")
    parser.add_argument("-w", "--weight", help="weight file",
                        default=f"{current_dir}/model_weight/yolo-fastest-xl_last.weights")
    parser.add_argument("-i", "--intrisic", help='camera intrinsic matrix',
                        default=f"{current_dir}/cfg/camera_param.yaml")
    parser.add_argument("-e", "--extrinsic", help="camera extrinsic matrix",
                        default=f"{current_dir}/cfg/calibration.yaml")
    parser.add_argument("-p" , "--predict" , help="inference flag" , default=True)

    return parser.parse_args()

def main():
    args = parser_args()
    
    camera_matrix = read_yaml_cv(args.intrisic , ["camera_matrix", "dist_matrix", "extrinsic_matrix"])
    extrinsic_matrix = read_yaml(args.extrinsic)
    camera_to_base_matrix = extrinsic_matrix['camera_to_base']

    depth_lst = glob(args.depth + "/*.png")
    color_lst = glob(args.color + "/*.png")
    depth_lst.sort()
    
    for depth_name in tqdm(depth_lst[1:2]):
        color_name = depth_name.replace('depth', 'color')
        if not os.path.exists(color_name):
            continue
        color_img = cv2.imread(color_name)
        depth_img = cv2.imread(depth_name , -1)
        color_img = cv2.cvtColor(color_img , cv2.COLOR_BGR2RGB)
        
        pcd = create_pointcloud_from_depth_and_rgb_cv(depth_img, color_img , camera_matrix['camera_matrix'] , None)
        print("before number: " , pcd)
        pcd = pcd.remove_non_finite_points()
        print("after number: " , pcd)
        vis_pointcloud(pcd , save_point_cloud="test.pcd")
        
        


if __name__ == '__main__':
    main()
    
