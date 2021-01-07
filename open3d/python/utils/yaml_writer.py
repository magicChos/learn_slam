#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :yaml_writer.py
@brief       :写入yaml接口
@time        :2021/01/08 00:50:50
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''

import yaml

def write_yaml(yaml_file, info):
    with open(yaml_file, "w") as f:
        yaml.dump(info, f, indent=4)
