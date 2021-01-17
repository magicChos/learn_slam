#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :color_img_data.py
@brief       :封装image的类
@time        :2021/01/11 16:45:42
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


class ImgData(object):
    def __init__(self, cv_img, time_stamp=0.0):
        self.time_stamp = time_stamp
        self.img = cv_img

    def image_info(self):
        return self.img

    def time_info(self):
        return self.time_stamp
