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
import sys
import traceback

from carline.draw_car_line import DrawCarLine
from utils.yaml_reader import  read_yaml_cv , read_yaml
from utils.cloud_filter import custom_filter
from utils.visualier import display_inlier_outlier
from utils.geometry_utils import createMap

sys.path.append("yoloapi")
from make_predict import YoloFastestModel

root_dir = "/home/han/sub_tof_data"
current_dir = os.getcwd()

def vis_pointcloud(points_3d, camera_2_base = None ,save_point_cloud=None):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_3d)
    
    if camera_2_base != None:
        point_cloud.transform(camera_2_base)
    point_cloud.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    alignbbox = point_cloud.get_axis_aligned_bounding_box()
    
    print("alignbbox info: " , alignbbox.get_print_info())
    
    alignbbox.color = (1, 0, 0)
    # orientdbbox = point_cloud.get_oriented_bounding_box()
    # orientdbbox.color = (0, 1, 0)
    
    o3d.visualization.draw_geometries(
        [point_cloud, alignbbox], width=600, height=600)

    if save_point_cloud != None:
        o3d.io.write_point_cloud(save_point_cloud, point_cloud)


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
    pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
  
    
    return pcd

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


def create_pointcloud_from_depth_and_rgb_roi(depth_map, cam_matrix, detections , flatten=False, depth_scale=1000):
    fx, fy = cam_matrix[0, 0], cam_matrix[1, 1]
    cx, cy = cam_matrix[0, 2], cam_matrix[1, 2]
    
    total_xyz = []
    for det in detections:

        start_x , start_y , end_x , end_y = det[2]
        h , w = np.mgrid[start_y : end_y , start_x : end_x]
        scale_depth_map = depth_map[start_y : end_y , start_x : end_x]/depth_scale
        
        x = (w-cx)*scale_depth_map/fx
        y = (h-cy)*scale_depth_map/fy
        
        xyz = np.dstack((x, y, scale_depth_map)) if flatten == False else np.dstack(
        (x, y, scale_depth_map)).reshape(-1, 3)
        
        z_filter = np.logical_and(xyz[: , 2] > 0.3 , xyz[: , 2] < 1.5)
        indices = np.argwhere(z_filter).flatten()
        total_xyz.append(xyz[indices].tolist())
        
    return total_xyz


def write_pcd(pcd, save_pcd_name):
    """保存点云数据."""
    o3d.io.write_point_cloud(save_pcd_name, pcd)


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
    
    draw_car_line_object = DrawCarLine(camera_matrix)
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    axis_pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    model = None
    
    if args.predict:
        model = YoloFastestModel(args.config_file, args.data_file, args.weight)  
    
    for depth_name in tqdm(depth_lst):
        print(depth_name)
        try:
            color_name = depth_name.replace('depth', 'color')
            if not os.path.exists(color_name):
                continue
            color_img = cv2.imread(color_name)
            depth_img = cv2.imread(depth_name , -1)
            color_img_RGB = cv2.cvtColor(color_img , cv2.COLOR_BGR2RGB)
            
            draw_geometry = []
            
            
            
            if model is not None:
                color_img , detections = model.predict_cv(color_img)
                roi_bboxes = []
                roi_pcds = create_pointcloud_from_depth_and_rgb_roi(depth_img , camera_matrix['camera_matrix'] , detections , flatten=True)
                for roi_pcd in roi_pcds:
                    
                    roi_cloud = o3d.geometry.PointCloud()
                    roi_cloud.points = o3d.utility.Vector3dVector(roi_pcd)
                    roi_cloud.transform(camera_to_base_matrix)
                    roi_cloud.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    print("before filter: " , roi_cloud)
                    indices = custom_filter(roi_cloud , [-0.3 , 0.3] , [0.3 , 2.0] , [0.03 , 0.5])
                    inlier_cloud = roi_cloud.select_by_index(indices)
                    
                    inlier_cloud , _ = inlier_cloud.remove_radius_outlier(10 , 0.01)
                    
                    
                    print("after filter: " , inlier_cloud)
                    
                    alignbbox = inlier_cloud.get_axis_aligned_bounding_box()
                    alignbbox.color = (0, 1, 0)
                    draw_geometry.append(alignbbox)
                    
                    max_bbox = alignbbox.get_max_bound().tolist()[:2]
                    min_bbox = alignbbox.get_min_bound().tolist()[:2]
                    
                    roi_bboxes.append((min_bbox , max_bbox))
                
            draw_car_line_object.drawline(color_img)
            tmp = create_pointcloud_from_depth_and_rgb_cv(depth_img, color_img_RGB , camera_matrix['camera_matrix'] , camera_to_base_matrix)
            
            # if len(roi_bboxes) > 0:
            #     map_img = createMap(tmp , res=0.005 , bboxes = roi_bboxes)
            #     cv2.imshow("map" , map_img)
                
            map_img = createMap(tmp , res=0.005 , bboxes = roi_bboxes)
            cv2.imshow("map" , map_img)    
            
            
            # save_pcd_name = os.path.basename(depth_name).replace("png" , "pcd")
            # write_pcd(tmp , save_pcd_name)
            
            indices = custom_filter(tmp , [-0.3 , 0.3] , [0.3 , 2.0] , [0.03 , 1])
            inlier_cloud = tmp.select_by_index(indices)
            outlier_cloud = tmp.select_by_index(indices , invert=True)
            outlier_cloud.paint_uniform_color([1, 0, 0])
            # inlier_cloud.paint_uniform_color([0, 1, 0])
            
            # draw_geometry.append(tmp)
            draw_geometry.append(inlier_cloud)
            draw_geometry.append(outlier_cloud)
            draw_geometry.append(axis_pcd)
            
            cv2.imshow("color" , color_img)
            cv2.waitKey(100)
            o3d.visualization.draw_geometries(draw_geometry , width=640, height=480)
            # cv2.destroyAllWindows()
            roi_bboxes = []
        except BaseException as e:
            print("{} occur problem: {}".format(depth_name , e))
            print(traceback.format_exc())

if __name__ == '__main__':
    main()
