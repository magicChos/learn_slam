#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :subscriber_image.py
@brief       :订阅sensor_msgs/PointCloud topic
@time        :2021/08/02 14:51:20
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import pdb
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from multiprocessing import JoinableQueue
import multiprocessing
import time


class CloudSubscriber(object):
    def __init__(self, topic_name, buffer_size=1):
        self.cloudSub = rospy.Subscriber(
            topic_name, PointCloud, self.msg_callback, queue_size=buffer_size)

    def msg_callback(self, data):
        try:
            import pdb
            pdb.set_trace()
            print(data.header.stamp.to_sec())
        except BaseException as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node("subscirbeCloud", anonymous=True)
    cloud_subscriber_obj = CloudSubscriber("/pico_camera/point_cloud")
    rospy.spin()
