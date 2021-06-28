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



def createMap(pcd, res=0.005, bboxes=None):
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    xyz = np.asarray(voxel_down_pcd.points)

    x_points = xyz[:, 0]
    y_points = xyz[:, 1]
    z_points = xyz[:, 2]

    z_min = z_points.min()
    z_max = z_points.max()

    # pixel_values = np.clip(a=z_points,
    #                        a_min=z_min,
    #                        a_max=z_max)
    # pixel_values = scale_to_255(pixel_values, min=z_min, max=z_max)
    x_img = (x_points/res).astype(np.int32)
    y_img = (y_points/res).astype(np.int32)

    x_min = x_img.min()
    x_max = x_img.max()
    y_min = y_img.min()
    y_max = y_img.max()

    width = 1 + x_max - x_min
    height = 1 + y_max
    im = np.zeros([height, width], dtype=np.uint8)

    x_img -= x_min
    y_img = y_max - y_img

    # im[y_img, x_img] = pixel_values

    if bboxes is not None:
        for box_min, box_max in bboxes:
            roi_xmin = (box_min[0]/res)
            roi_ymin = (box_min[1]/res)
            roi_xmax = (box_max[0]/res)
            roi_ymax = (box_max[1]/res)

            pt_x_min = int(roi_xmin - x_min)
            pt_x_max = int(roi_xmax - x_min)
            pt_y_min = int(y_max - roi_ymax)
            pt_y_max = int(y_max - roi_ymin)

            cv2.rectangle(im, (pt_x_min, pt_y_min),
                          (pt_x_max, pt_y_max), (255, 0, 0), 1)

    return im


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud(
        "/home/han/Desktop/00088_2021-01-07_16-04-58_165.pcd")
    
    bboxes = [([-0.0125, 0.724] , [0.0909, 0.794])]
    im = createMap(pcd , res = 0.005 , bboxes= bboxes)

    cv2.imshow("image", im)
    cv2.waitKey(0)
