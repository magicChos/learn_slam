#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :subscriber_image.py
@brief       :订阅图像topic
@time        :2021/01/08 14:51:20
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
from multiprocessing import JoinableQueue
import multiprocessing
import sys
import os
from collections import deque

sys.path.remove("/opt/ros/melodic/lib/python2.7/dist-packages")

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__) , "../..")))
from sensor_data.image_data import ImgData



class ImageSubscriber(object):
    def __init__(self  , topic_name , buffer_size = 1 , depth_flag = False , model = None ,draw_car_line_obj = None , show_result = True):
        '''
        model: 传过来的yolo检测模型
        draw_car_line_obj: 绘制车道线对象
        '''
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(topic_name , Image , self.msg_callback , queue_size=buffer_size)
        self.q = deque()
        self.count = 0
        self.model = model
        self.draw_car_line_obj = draw_car_line_obj
        self.depth_flag = depth_flag
        self.show_result = show_result
    
    def msg_callback(self , data):
        try:
            if self.depth_flag:
                image_raw = self.bridge.imgmsg_to_cv2(data , "16UC1")
            else:
                image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            

            # print(float(data.header.stamp.to_sec()))
            
            if self.model is not None:
                image_raw, detections = self.model.predict_cv(image_raw)
                
            if self.draw_car_line_obj is not None:
                self.draw_car_line_obj.drawline(image_raw)
                
            image_data_obj = ImgData(image_raw , time_stamp=data.header.stamp.to_sec())
 
            self.q.append(image_data_obj)
            # save_name = str(self.count) + ".jpg"

            if self.show_result: 
                cv2.imshow("image" , image_raw)
                cv2.waitKey(25)
                
            self.count += 1
            
        except CvBridgeError as e:
            print(e)
            
    def parse_data(self):
        return self.q
        

if __name__ == '__main__':
    rospy.init_node("subscirbeImage" , anonymous=True)
    img_subscriber_obj = ImageSubscriber("/pico_camera/color_image" , buffer_size=100 , show_result=False)
    
    rate = rospy.Rate(10)
    while True:
        image_q = img_subscriber_obj.parse_data()
        if image_q:
            image_data = image_q.popleft()
            image_raw = image_data.image_info()
            cv2.imshow("image" , image_raw)
            cv2.waitKey(25)

        
        rate.sleep()
        
    rospy.spin()
    