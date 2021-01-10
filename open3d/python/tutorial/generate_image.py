#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :generate_image.py
@brief       :生成高程值影像
@time        :2021/01/09 11:09:52
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''


import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt


def scale_to_255(a, min, max, dtype=np.uint8):
    """Scales an array of values from specified min, max range to 0-255
    Optionally specify the data type of the output (default is uint8)"""
    return (((a - min) / float(max - min)) * 255).astype(dtype)


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("/home/han/Desktop/save.pcd")
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    xyz = np.asarray(voxel_down_pcd.points)
    
    x_points = xyz[:,0]
    y_points = xyz[:,1]
    z_points = xyz[:,2]
    
    z_min = z_points.min()
    z_max = z_points.max()
    
    pixel_values = np.clip(a = z_points,
                           a_min=z_min,
                           a_max=z_max)
    pixel_values  = scale_to_255(pixel_values, min=z_min, max=z_max)
    
    res = 0.005
   
    
    x_img = (x_points/res).astype(np.int32)
    y_img = (y_points/res).astype(np.int32)
    
    x_min = x_img.min()
    x_max = x_img.max()
    y_min = y_img.min()
    y_max = y_img.max()
    
    width = 1 + x_max - x_min
    height = 1 + y_max
    im = np.zeros([height, width], dtype=np.uint8)
    
    x_img -=x_min
    y_img = y_max - y_img  
    
    im[y_img , x_img] = pixel_values

    cv2.imshow("image" , im)
    cv2.waitKey(0)
    
    
    
