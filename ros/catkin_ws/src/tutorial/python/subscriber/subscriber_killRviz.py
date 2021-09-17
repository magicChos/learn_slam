#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :subscriber_killRviz.py
@brief       :接收信号杀掉rviz
@time        :2021/09/16 22:18:49
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
Copyright (c) 2020. All rights reserved.Created by hanshuo
'''



import rospy
from std_msgs.msg import Int32
import os


class KillRviz(object):
    def __init__(self, topic_name, buffer_size=1):
        self.cloudSub = rospy.Subscriber(
            topic_name, Int32, self.msg_callback, queue_size=buffer_size)

    def msg_callback(self, data):
        try:
            if data.data == 1:
                nodes = os.popen("rosnode list").readlines()
                for i in range(len(nodes)):
                    nodes[i] = nodes[i].replace("\n", "")
                    if "rviz" in nodes[i]:
                        os.system("rosnode kill " + nodes[i])

        except BaseException as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node("killRviz", anonymous=True)
    cloud_subscriber_obj = KillRviz("/kill/rviz")
    rospy.spin()
