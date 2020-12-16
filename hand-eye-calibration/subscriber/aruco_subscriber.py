#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :aruco_subscriber.py
@brief       :订阅/aruco_single/transform
@time        :2020/12/16 15:22:45
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''

import rospy
import math
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class ArucoSub(object):
    def __init__(self , aruco_trans_topic):
        self.aruco_trans_topic = aruco_trans_topic
        self.trans_pub = rospy.Subscriber(aruco_trans_topic , TransformStamped , self.callback)
        self.queue = []
        
    def callback(self , data):
        
        translation = data.transform.translation
        rotation = data.transform.rotation
        
        translation_vec = [translation.x , translation.y , translation.z]
        rotation_vec = [rotation.x , rotation.y , rotation.z , rotation.w]
        
        R = {"tran": translation_vec , "quart": rotation_vec}
        self.queue.append(R)
        
        
        if len(self.queue) < 10:
            pass
        else:
            mean_translation_x = 0.0
            mean_translation_y = 0.0
            mean_translation_z = 0.0
            mean_rotation_x = 0.0
            mean_rotation_y = 0.0
            mean_rotation_z = 0.0
            mean_rotation_w = 0.0
            for t in self.queue:
                mean_translation_x += t['tran'][0]
                mean_translation_y += t['tran'][1]
                mean_translation_z += t['tran'][2]
                
                mean_rotation_x += t['quart'][0]
                mean_rotation_y += t['quart'][1]
                mean_rotation_z += t['quart'][2]
                mean_rotation_w += t['quart'][3]
                
            
            mean_translation_x /= 10
            mean_translation_y /= 10
            mean_translation_z /= 10
            mean_rotation_x /= 10
            mean_rotation_y /= 10
            mean_rotation_z /= 10
            mean_rotation_w /= 10
            
            # 四元数转旋转矩阵
            rotation_matrix = Rotation([mean_rotation_x , mean_rotation_y , mean_rotation_z , mean_rotation_w]).as_matrix()
            T = np.identity(4 , dtype=np.float32)
            T[:3 , :3] = rotation_matrix
            T[0][3] = mean_translation_x
            T[1][3] = mean_translation_y
            T[2][3] = mean_translation_z
            
            print("T: " , T)
            self.queue = []
            
            
        
def main():
    rospy.init_node('arucoTransformNode')
    aruco_sub = ArucoSub('/aruco_single/transform')
    rospy.spin()
    q = aruco_sub.get_transformation()
    print(q)
        
if __name__ == '__main__':
    main()
    
        