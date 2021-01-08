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

class ImageSubscriber(object):
    def __init__(self  , topic_name , buffer_size = 1 , model = None):
        '''
        model: 传过来的yolo检测模型
        '''
        self.bridge = CvBridge()
        self.imageSub = rospy.Subscriber(topic_name , Image , self.msg_callback , queue_size=buffer_size)
        self.q = JoinableQueue()
        self.count = 0
        self.model = model
    
    def msg_callback(self , data):
        try:
            image_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.model is not None:
                image_raw, detections = self.model.predict_cv(image_raw)
            
            save_name = str(self.count) + ".jpg"
            # cv2.imwrite(save_name , image_raw)
            cv2.imshow("image" , image_raw)
            cv2.waitKey(25)
            self.count += 1
            
        except CvBridgeError as e:
            print(e)
            
    def parse_data(self , q):
        q = self.q
        print('parse q size: ' , q.qsize())
        

if __name__ == '__main__':
    rospy.init_node("subscirbeImage" , anonymous=True)
    
    image_q = JoinableQueue()
    img_subscriber_obj = ImageSubscriber("/pico_camera/color_image")
    rospy.spin()
    