#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :utils.py
@brief       :包含一些辅助函数
@time        :2020/12/13 15:46:10
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''

import numpy as np



def calcIOU(x1, y1, w1, h1, x2, y2, w2, h2):
    '''
    :param x1:center_x
    :param y1:center_y
    :param w1:
    :param h1:
    :param x2:
    :param y2:
    :param w2:
    :param h2:
    :return:
    '''
    IOU = 0
    if ((abs(x1 - x2) < ((w1 + w2) / 2.0)) and (abs(y1 - y2) < ((h1 + h2) / 2.0))):
        left = max((x1 - (w1 / 2.0)), (x2 - (w2 / 2.0)))
        upper = max((y1 - (h1 / 2.0)), (y2 - (h2 / 2.0)))

        right = min((x1 + (w1 / 2.0)), (x2 + (w2 / 2.0)))
        bottom = min((y1 + (h1 / 2.0)), (y2 + (h2 / 2.0)))
        inter_w = abs(left - right)
        inter_h = abs(upper - bottom)
        inter_square = inter_w * inter_h
        union_square = (w1 * h1) + (w2 * h2) - inter_square
        IOU = inter_square / union_square * 1.0

    return IOU


def iou_process(detections):
    iou_matrix = np.zeros((len(detections), len(detections)), dtype=np.float32)
    for i in range(len(detections)):
        cls_i, conf_i, (x_i, y_i, w_i, h_i) = detections[i]
        iou_matrix[i][i] = 0.0
        for j in range(i + 1, len(detections)):
            cls_j, conf_j, (x_j, y_j, w_j, h_j) = detections[j]
            iou = calcIOU(x_i, y_i, w_i, h_i, x_j, y_j, w_j, w_j)
            iou_matrix[i][j] = iou
            iou_matrix[j][i] = 0.0

    sel_index = np.where(iou_matrix > 0.2)
    del_lst = sel_index[1].tolist()
    if len(del_lst) == 0:
        detections = detections[:1]
    else:
        new_detections = []
        for i, d in enumerate(detections):
            if i in del_lst:
                continue
            new_detections.append(d)
        detections = new_detections

    return detections



