#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :make_predict.py
@brief       :前向传播
@time        :2020/12/13 01:51:59
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''


import argparse
import os
import glob
import pdb
import random
from random import shuffle
import time
import cv2
import numpy as np
# from yoloapi import darknet
import darknet
from darknet import bbox2points
# from yoloapi.darknet import bbox2points
from imutils.paths import list_images
import shutil
from tqdm import tqdm
# from pascal_voc_io import PascalVocWriter, XML_EXT
# from class_path import classes_lst as classes
# from yoloapi.pascal_voc_io import PascalVocWriter , XML_EXT
# from yoloapi.class_path import classes_lst as classes
from prettytable import PrettyTable
from utils_fun import calcIOU, iou_process
# from yoloapi.utils import calcIOU , iou_process


current_file_dir = os.path.dirname(os.path.abspath(__file__))
exe_file_dir = os.path.abspath(os.path.join(current_file_dir , ".."))


print("current_file_dir: " , current_file_dir)
print("exe_file_dir: " , exe_file_dir)

class YoloFastestModel(object):
    def __init__(self, cfg_file, data_file, model_file):
        self.cfg_file = cfg_file
        self.data_file = data_file
        self.model_file = model_file

        self.network = None
        self.class_names = None
        self.class_colors = None

        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.cfg_file,
            self.data_file,
            self.model_file,
            batch_size=1)
        
    

    def convert_real_detections(self, detections_det, ratio_val_h=1.0, ratio_val_w=1.0):
        detections_src = []
        for det in detections_det:
            cls, conf, bndbox = det
            left, top, right, bottom = bbox2points(bndbox)

            bndbox = (left * ratio_val_w, top * ratio_val_h,
                      right * ratio_val_w, bottom * ratio_val_h)
            bndbox = list(map(int, bndbox))
            detections_src.append((cls, conf, bndbox))
        return detections_src

    def image_detection(self, image_path, net, cls_names, thresh=0.6, save_annotations=False):
        # Darknet doesn't accept numpy images.
        # Create one with image we reuse for each detect
        width = darknet.network_width(net)
        height = darknet.network_height(net)
        darknet_image = darknet.make_image(width, height, 3)

        image = cv2.imread(image_path)
        if type(image) == type(None):
            return None , None
        
        img_h, img_w = image.shape[:2]
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(
            net, cls_names, darknet_image, thresh=thresh)

        if len(detections) > 0:
            detections = iou_process(detections)

        ratio_val_h = img_h/height
        ratio_val_w = img_w/width

        if save_annotations:
            if len(detections) == 0:
                pass
            else:
                writer = PascalVocWriter(os.path.dirname(image_path), os.path.basename(
                    image_path), image.shape, localImgPath=image_path, usrname="auto")
                writer.verified = True

                for label, confidence, bbox in detections:
                    left, top, right, bottom = bbox2points(bbox)
                    left = int(left * ratio_val_w)
                    top = int(top * ratio_val_h)
                    right = int(right * ratio_val_w)
                    bottom = int(bottom * ratio_val_h)
                    cv2.rectangle(image, (left, top),
                                  (right, bottom), (0, 0, 255), 2)
                    cv2.putText(image, "{} [{:.2f}]".format(label, float(confidence)),
                                (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 2)

                    writer.addBndBox(left, top, right, bottom, label, 0)
                writer.save(targetFile=image_path.replace(".jpg", ".xml"))

        else:
            for label, confidence, bbox in detections:
                left, top, right, bottom = bbox2points(bbox)
                left = int(left * ratio_val_w)
                top = int(top * ratio_val_h)
                right = int(right * ratio_val_w)
                bottom = int(bottom * ratio_val_h)
                cv2.rectangle(image, (left, top),
                              (right, bottom), (0, 0, 255), 2)
                cv2.putText(image, "{} [{:.2f}]".format(label, float(confidence)),
                            (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 0, 0), 2)

        detections = self.convert_real_detections(
            detections, ratio_val_h=ratio_val_h, ratio_val_w=ratio_val_w)
        return image, detections
    
    
    def image_detection_cv(self, cv_img, net, cls_names, thresh=0.6):
        """输入为cv的image."""
        width = darknet.network_width(net)
        height = darknet.network_height(net)
        darknet_image = darknet.make_image(width, height, 3)
        
        img_h, img_w = cv_img.shape[:2]
        image_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (width, height),
                                   interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(
            net, cls_names, darknet_image, thresh=thresh)

        if len(detections) > 0:
            detections = iou_process(detections)

        ratio_val_h = img_h/height
        ratio_val_w = img_w/width

        
        for label, confidence, bbox in detections:
            left, top, right, bottom = bbox2points(bbox)
            left = int(left * ratio_val_w)
            top = int(top * ratio_val_h)
            right = int(right * ratio_val_w)
            bottom = int(bottom * ratio_val_h)
            cv2.rectangle(cv_img, (left, top), (right, bottom), (0, 0, 255), 2)
            cv2.putText(cv_img, "{} [{:.2f}]".format(label, float(confidence)),
                            (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 0, 0), 2)

        detections = self.convert_real_detections(
            detections, ratio_val_h=ratio_val_h, ratio_val_w=ratio_val_w)
        return cv_img, detections

    def predict(self, image_name, thresh=0.6):
        image, detections = self.image_detection(
            image_name, self.network, self.class_names, thresh)

        return image, detections
    
    def predict_cv(self, cv_img, thresh=0.6):
        image, detections = self.image_detection_cv(
            cv_img, self.network, self.class_names, thresh)

        return cv_img, detections


def parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config_file", default=f"{exe_file_dir}/cfg/yolo-fastest-xl.cfg",
                        help="path to config file")
    parser.add_argument("-d", "--data_file", default=f"{exe_file_dir}/cfg/voc.data",
                        help="path to data file")
    parser.add_argument("-t", "--thresh", type=float, default=.1,
                        help="remove detections with lower confidence")
    parser.add_argument("-w", "--weight", help="weight file",
                        default=f"{exe_file_dir}/model_weight/yolo-fastest-xl_last.weights")
    parser.add_argument("-i", "--input", help="input image dir",
                        default="/home/han/Desktop/tof_data/color")

    return parser.parse_args()


def main():
    args = parser_args()

    model = YoloFastestModel(args.config_file, args.data_file, args.weight)

    image_lst = glob.glob(args.input + "/*.png")
    image_lst.sort()
    for name in tqdm(image_lst):
        image, detections = model.predict(name)
        cv2.imshow("image", image)
        key = cv2.waitKey(20)
        if key == ord('q') & 0xff:
            break 


if __name__ == '__main__':
    main()
