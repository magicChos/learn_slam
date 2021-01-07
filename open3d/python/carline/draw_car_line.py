#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :draw_car_line.py
@brief       :绘制车道线
@time        :2020/11/11 01:01:59
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import cv2
from glob import glob
import os
import shutil
import argparse
import numpy as np
from tqdm import tqdm


def createCarline():
    '''
    创建车道线
    '''
    left_car_line = []
    right_car_line = []
    
    left_car_line.append([0.144 , 0.1 , 0.1])
    right_car_line.append([-0.206, 0.1, 0.1])
    
    for i in range(1 , 5):
        left_car_line.append([0.144 , 0.1 , 0.5 * i])
        right_car_line.append([-0.206, 0.1 , 0.5 * i])
        
    return left_car_line , right_car_line

def drawCarline():
    '''
    绘制车道线
    '''


def read_yaml(yaml_file):
    res = None
    with open(yaml_file, "r") as f:
        res = yaml.load(f)
    return res


def parse_calibrate_result(yaml_res):
    # camera to lidar geometry transformation , 4 x 4
    T = yaml_res['CameraExtrinsicMat']['data']
    # camera distcoeff
    K = yaml_res['DistCoeff']['data']
    camera_intrinsic = yaml_res['CameraMat']['data']

    T = np.matrix(T).reshape(4, 4)
    R_cl = T[0:3, :3]
    T_cl = T[0:3, 3]
    K = np.array(K)
    C = np.array([camera_intrinsic[2], camera_intrinsic[5]])
    F = np.array([camera_intrinsic[0], camera_intrinsic[4]])

    R_lc = R_cl.T
    T_lc = -R_lc * T_cl

    all_calibration_info = {}
    # 畸变系数
    all_calibration_info['K'] = K.tolist()
    # 像主点坐标
    all_calibration_info['C'] = C.tolist()
    # 焦距信息
    all_calibration_info['F'] = F.tolist()
    # lidar到相机的旋转矩阵
    all_calibration_info['R'] = R_lc.tolist()
    # lidar到相机的平移向量
    all_calibration_info['T'] = T_lc.tolist()

    return all_calibration_info


def project_Camera_3d_to_image(camera_3d_pt, calibration_info):
    '''
    将摄像机坐标系下的三维点投影到图像上
    '''
    # 自定义投影
    # camera_3d_pt[:, :2] /= camera_3d_pt[:, 2]
    # a = camera_3d_pt[:, 0]
    # b = camera_3d_pt[:, 1]

    # K = np.array(calibration_info['K'])
    # C = np.array(calibration_info['C'])
    # F = np.array(calibration_info['F'])

    # r = np.sqrt(np.power(a, 2) + np.power(b, 2))

    # ad = np.multiply(a, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4))))
    # bd = np.multiply(b, (1 + np.multiply(K[0], np.power(r, 2)) + np.multiply(K[1], np.power(r, 4))))

    # pixel_x = (F[0] * a + C[0])
    # pixel_y = (F[1] * b + C[1])
    # pixel_list = np.hstack((pixel_x, pixel_y))

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
        camera_3d_pt, rotate_vec, translation_vec , camera_matrix , dist_coef)
    return pixel_list


def project_Camera_3d_to_image(camera_3d_pt, calibration_info):
    # to do
    pass



def draw_car_line(cv_img, left_line_pts, right_line_pts):
    left_start_x, left_start_y = left_line_pts[0].tolist()[0]
    left_end_x, left_end_y = left_line_pts[-1].tolist()[0]
    right_start_x, right_start_y = right_line_pts[0].tolist()[0]
    right_end_x, right_end_y = right_line_pts[-1].tolist()[0]

    cv2.line(cv_img, (int(left_start_x), int(left_start_y)),
             (int(left_end_x), int(left_end_y)), (0, 0, 255), 2)
    cv2.line(cv_img, (int(right_start_x), int(right_start_y)),
             (int(right_end_x), int(right_end_y)), (0, 0, 255), 2)

    assert left_line_pts.shape == right_line_pts.shape, print(
        "left line pts must be same with right line pts")
    # import pdb; pdb.set_trace()
    for i in range(left_line_pts.shape[0]):
        left_pt_x, left_pt_y = left_line_pts[i].tolist()[0]
        right_pt_x, right_pt_y = right_line_pts[i].tolist()[0]

        cv2.line(cv_img, (int(left_pt_x), int(left_pt_y)),
                 (int(right_pt_x), int(right_pt_y)), (0, 0, 255), 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="input image dir")
    parser.add_argument("-c", "--calib_config",
                        help="calibration file", default="9-27-result-test.yaml")
    parser.add_argument("-s", "--save", help='save dir', default="save_result")
    args = parser.parse_args()

    if os.path.exists(args.save):
        shutil.rmtree(args.save)
        os.mkdir(args.save)
    else:
        os.mkdir(args.save)

    image_lst = glob(args.input + "/*.jpg")
    res = read_yaml(args.calib_config)
    calibration_info = parse_calibrate_result(res)

    left_car_line = []
    right_car_line = []
    left_car_line.append([0.145, 0.1, 0.1])
    left_car_line.append([0.145, 0.1, 0.5])
    left_car_line.append([0.145, 0.1, 1.0])
    left_car_line.append([0.145, 0.1, 1.5])
    left_car_line.append([0.145, 0.1, 2.0])

    right_car_line.append([-0.145, 0.1, 0.1])
    right_car_line.append([-0.145, 0.1, 0.5])
    right_car_line.append([-0.145, 0.1, 1.0])
    right_car_line.append([-0.145, 0.1, 1.5])
    right_car_line.append([-0.145, 0.1, 2.0])

    left_car_line = np.matrix(left_car_line, dtype=np.float32)
    right_car_line = np.matrix(right_car_line, dtype=np.float32)

    left_line_pts = project_Camera_3d_to_image(left_car_line, calibration_info)
    right_line_pts = project_Camera_3d_to_image(
        right_car_line, calibration_info)

    for name in tqdm(image_lst):
        img = cv2.imread(name)
        draw_car_line(img, left_line_pts, right_line_pts)

        new_name = os.path.join(args.save, os.path.basename(name))
        cv2.imwrite(new_name, img)


if __name__ == '__main__':
    main()
