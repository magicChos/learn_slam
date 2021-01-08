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

import sys
from tqdm import tqdm
import numpy as np
import argparse
import shutil
import cv2
from glob import glob
import os

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from utils.yaml_reader import read_yaml_cv
from utils.geometry_utils import project_Camera_3d_to_image


class DrawCarLine(object):
    def __init__(self, calib_info):
        self.left_car_line, self.right_car_line = self.createCarline()
        self.left_car_line = np.matrix(self.left_car_line, dtype=np.float32)
        self.right_car_line = np.matrix(self.right_car_line, dtype=np.float32)
        self.calibration_info = self.parse_calibrate_result(calib_info)
        self.left_car_pts = project_Camera_3d_to_image(
            self.left_car_line, self.calibration_info)
        self.right_car_pts = project_Camera_3d_to_image(
            self.right_car_line, self.calibration_info)

    def createCarline(self):
        '''
        创建车道线
        '''
        left_car_line = []
        right_car_line = []
        left_car_line.append([0.144, 0.1, 0.1])
        right_car_line.append([-0.206, 0.1, 0.1])

        for i in range(1, 5):
            left_car_line.append([0.144, 0.1, 0.5 * i])
            right_car_line.append([-0.206, 0.1, 0.5 * i])

        return left_car_line, right_car_line

    def parse_calibrate_result(self, yaml_res):
        # camera to lidar geometry transformation , 4 x 4
        T = yaml_res['extrinsic_matrix']

        # camera distcoeff
        K = yaml_res['dist_matrix'].flatten()
        camera_intrinsic = yaml_res['camera_matrix']

        T = np.matrix(T).reshape(4, 4)
        R_cl = T[0:3, :3]
        T_cl = T[0:3, 3]
        K = np.array(K)
        C = np.array([camera_intrinsic[0][2], camera_intrinsic[1][2]])
        F = np.array([camera_intrinsic[0][0], camera_intrinsic[1][1]])

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

    def drawline(self, cv_img):
        left_start_x, left_start_y = self.left_car_pts[0].tolist()[0]
        left_end_x, left_end_y = self.left_car_pts[-1].tolist()[0]
        right_start_x, right_start_y = self.right_car_pts[0].tolist()[0]
        right_end_x, right_end_y = self.right_car_pts[-1].tolist()[0]

        cv2.line(cv_img, (int(left_start_x), int(left_start_y)),
                 (int(left_end_x), int(left_end_y)), (0, 0, 255), 2)
        cv2.line(cv_img, (int(right_start_x), int(right_start_y)),
                 (int(right_end_x), int(right_end_y)), (0, 0, 255), 2)

        assert self.left_car_pts.shape == self.right_car_pts.shape, print(
            "left line pts must be same with right line pts")

        for i in range(self.left_car_pts.shape[0]):
            left_pt_x, left_pt_y = self.left_car_pts[i].tolist()[0]
            right_pt_x, right_pt_y = self.right_car_pts[i].tolist()[0]

            cv2.line(cv_img, (int(left_pt_x), int(left_pt_y)),
                     (int(right_pt_x), int(right_pt_y)), (0, 0, 255), 1)

        return cv_img
    
    def getCarlineInfo(self):
        '''
        获取车道线信息
        '''
        return self.left_car_pts , self.right_car_pts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="input image dir",
                        default="/home/han/tof_data/color")
    parser.add_argument("-c", "--calib_config",
                        help="calibration file", default="/home/han/tof_data/camera_param.yaml")
    parser.add_argument("-s", "--save", help='save dir', default="save_result")
    args = parser.parse_args()

    if os.path.exists(args.save):
        shutil.rmtree(args.save)
        os.mkdir(args.save)
    else:
        os.mkdir(args.save)

    image_lst = glob(args.input + "/*.png")
    image_lst.sort()
    res = read_yaml_cv(args.calib_config, [
                       "camera_matrix", "dist_matrix", "extrinsic_matrix"])

    car_line_object = DrawCarLine(res)

    for name in tqdm(image_lst):
        img = cv2.imread(name)
        car_line_object.drawline(img)

        new_name = os.path.join(args.save, os.path.basename(name))
        cv2.imshow("image", img)
        key = cv2.waitKey(0)
        if key == ord('q') & 0xff:
            break


if __name__ == '__main__':
    main()
