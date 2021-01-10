#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :cloud_outlier_removal.py
@brief       :
@time        :2021/01/09 00:56:38
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :open3d点云滤波
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''

import open3d as o3d
import numpy as np
import os
import sys




def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    
    
def custom_filter(pcd , x_range = [-1 , 1]  , y_range = [-1 , 1] , z_range = [1 , 1]):  
    xyz = np.asarray(pcd.points)
    x_points = xyz[:, 0]
    y_points = xyz[:, 1]
    z_points = xyz[:, 2]
    
    x_filter = np.logical_and((x_points > x_range[0]) , (x_points < x_range[1]))
    y_filter = np.logical_and((y_points > y_range[0]) , (y_points < y_range[1]))
    z_filter = np.logical_and((z_points > z_range[0]) , (z_points < z_range[1]))
    
    filter = np.logical_and(np.logical_and(x_filter , y_filter) , z_filter)
    indices = np.argwhere(filter).flatten()
    
    return indices
    
    


if __name__ == "__main__":
    
    # voxel_downsample
    pcd = o3d.io.read_point_cloud("/home/han/Desktop/save.pcd")
    # o3d.visualization.draw_geometries([pcd])
    print("before voxel filter point cloud number: " , pcd)
    
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    # o3d.visualization.draw_geometries([voxel_down_pcd])
    print("after voxel filter point cloud number: " , voxel_down_pcd)
    
    # uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
    # o3d.visualization.draw_geometries([uni_down_pcd])
    # print("after uniform_down_sample filter point cloud number: " , uni_down_pcd)
    
    
    # print("Statistical oulier removal")
    # cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,
    #                                                     std_ratio=2.0)
    # display_inlier_outlier(voxel_down_pcd, ind)
    
    # print("Radius oulier removal")
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    # display_inlier_outlier(voxel_down_pcd, ind)
    
    # ransac
    # plane_model , inliers = voxel_down_pcd.segment_plane(distance_threshold=0.1,
    #                                      ransac_n=10,
    #                                      num_iterations=1000)
    # [a , b , c , d] = plane_model
    # print(plane_model)

    # 设置感兴趣区域
    x_range = [-0.5 , 0.5]
    y_range = [0 , 2.0]
    z_range = [0.0 , 0.05]
    
    indices = custom_filter(cl , x_range = x_range, y_range = y_range , z_range = z_range)
    
    display_inlier_outlier(cl , indices)
    
   
    
    
    
    