#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :yaml_reader.py
@brief       :yaml读取相关接口
@time        :2021/01/08 00:47:37
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''
import cv2
import yaml

def read_yaml_cv(yml_file , node_name_list):
    fs = cv2.FileStorage(yml_file, cv2.FILE_STORAGE_READ)
    mat_info = {}
    for name in node_name_list:
        fn = fs.getNode(name)
        mat_info[name] = fn.mat()
    
    return mat_info

def read_yaml(yaml_file):
    res = None
    with open(yaml_file, "r") as f:
        res = yaml.load(f)
    return res



