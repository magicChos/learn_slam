#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :cloud_filter.py
@brief       :点云滤波相关的api
@time        :2021/01/10 22:49:37
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''

import numpy as np

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