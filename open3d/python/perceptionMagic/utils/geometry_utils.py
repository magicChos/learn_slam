#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :geometry_utils.py
@brief       :包含一些几何相关的接口
@time        :2021/01/08 12:02:57
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import numpy as np
import cv2
from utils.cloud_filter import custom_filter


def project_Camera_3d_to_image(camera_3d_pt, calibration_info):
    '''
    将摄像机坐标系下的三维点投影到图像上
    '''
    # 自定义投影
    camera_3d_pt[:, :2] /= camera_3d_pt[:, 2]
    a = camera_3d_pt[:, 0]
    b = camera_3d_pt[:, 1]

    K = np.array(calibration_info['K'])
    C = np.array(calibration_info['C'])
    F = np.array(calibration_info['F'])

    r = np.sqrt(np.power(a, 2) + np.power(b, 2))

    # ad = np.multiply(
    #     a, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4))))
    # bd = np.multiply(
    #     b, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4))))

    pixel_x = (F[0] * a + C[0])
    pixel_y = (F[1] * b + C[1])

    pixel_list = np.hstack((pixel_x, pixel_y))

    return pixel_list


def project_Camera_3d_to_image_cv(camera_3d_pt, calibration_info):
    # 调用opencv中的投影方法
    rotate_vec = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
    translation_vec = np.array([0.0, 0.0, 0.0]).reshape(1, 3)

    camera_matrix = np.identity(3, dtype=np.float32)
    camera_matrix[0][0] = calibration_info['F'][0]
    camera_matrix[1][1] = calibration_info['F'][1]
    camera_matrix[0][2] = calibration_info['C'][0]
    camera_matrix[1][2] = calibration_info['C'][1]

    dist_coef = np.zeros(4)
    dist_coef[0] = calibration_info['K'][0]
    dist_coef[1] = calibration_info['K'][1]
    dist_coef[2] = calibration_info['K'][2]
    dist_coef[3] = calibration_info['K'][3]

    pixel_list, _ = cv2.projectPoints(
        camera_3d_pt, rotate_vec, translation_vec, camera_matrix, dist_coef)
    return pixel_list


def scale_to_255(a, min, max, dtype=np.uint8):
    """Scales an array of values from specified min, max range to 0-255
    Optionally specify the data type of the output (default is uint8)"""
    return (((a - min) / float(max - min)) * 255).astype(dtype)


def createMap(pcd, x_range=[-0.4, 0.4], y_range=[0, 1.5], z_range=[0.003, 0.5], res=0.005, bboxes=None):
    # voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    # xyz = np.asarray(voxel_down_pcd.points)

    indices = custom_filter(pcd, x_range=x_range,
                            y_range=y_range, z_range=z_range)
    xyz = pcd.select_by_index(indices)
    xyz = np.asarray(xyz.points)

    x_points = xyz[:, 0]
    y_points = xyz[:, 1]
    z_points = xyz[:, 2]

    # z_min = z_points.min()
    # z_max = z_points.max()

    # pixel_values = np.clip(a=z_points,
    #                        a_min=z_min,
    #                        a_max=z_max)
    # pixel_values = scale_to_255(pixel_values, min=z_min, max=z_max)
    
    x_img = (x_points/res).astype(np.int32)
    y_img = (y_points/res).astype(np.int32)

    # x_min = x_img.min()
    # x_max = x_img.max()
    # y_min = y_img.min()
    # y_max = y_img.max()

    x_min = int(x_range[0]/res)
    x_max = int(x_range[1]/res)
    y_min = int(y_range[0]/res)
    y_max = int(y_range[1]/res)

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
    print(im.shape)
    return im
