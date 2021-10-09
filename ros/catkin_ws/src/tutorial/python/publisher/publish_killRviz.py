#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :ImagePublisher.py
@brief       :发送kill rviz信号
@time        :2021/01/08 16:22:59
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


from time import sleep
import rospy
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
from std_msgs.msg import Header
from glob import glob
from std_msgs.msg import Int32


class KillRvizPublisher(object):
    def __init__(self, top_name, buffer_size=1):
        self.publisher = rospy.Publisher(
            top_name, Int32, queue_size=buffer_size)

    def publish_kill(self, data):
        sleep(1)
        self.publisher.publish(data)


# 读取摄像头发布image
def main():
    rospy.init_node("publish_image", anonymous=True)
    kill_publisher_obj = KillRvizPublisher("/kill/rviz")
    kill_publisher_obj.publish_kill(Int32(1)) 


if __name__ == '__main__':
    main()
