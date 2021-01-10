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
