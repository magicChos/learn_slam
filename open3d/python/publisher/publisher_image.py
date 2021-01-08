#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :ImagePublisher.py
@brief       :发布图像数据
@time        :2021/01/08 16:22:59
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


import rospy
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
from std_msgs.msg import Header
from glob import glob


class ImagePublisher(object):
    def __init__(self, top_name, buffer_size=1):
        self.publisher = rospy.Publisher(
            top_name, Image, queue_size=buffer_size)

    def publish_image(self, img_data):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = "color_camera"

        height, width = img_data.shape[:2]
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'rgb8'
        image_temp.data = np.array(img_data).tostring()
        image_temp.header = header
        image_temp.step = width * 3
        
        self.publisher.publish(image_temp)


if __name__ == '__main__':
    rospy.init_node("publish_image", anonymous=True)
    image_lst = glob("/home/han/data/project/learn_slam/ros/catkin_ws/src/tutorial/python/subscriber" + "/*.jpg")
    image_lst = sorted(image_lst , key=lambda item : int(os.path.basename(item)[:-4]))
    
    img_publisher_obj = ImagePublisher("camera/color_image_raw")
    
    rate = rospy.Rate(10)
    
    for name in image_lst:
        print(name)
        img = cv2.imread(name)
        print("img shape: " , img.shape)
        img_publisher_obj.publish_image(img)
        rate.sleep()
        
        
        