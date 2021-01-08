#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@filename    :perception_detect_node.py
@brief       :感知模块检测结点
@time        :2021/01/08 17:08:51
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
'''


from subscriber.subscriber_image import ImageSubscriber
from publisher.publisher_image import ImagePublisher
import argparse
import os
import rospy
import sys

sys.path.append("yoloapi")
from make_predict import YoloFastestModel


exe_file_dir = os.getcwd()

model = None

def parser_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config_file", default="{}/cfg/yolo-fastest-xl.cfg".format(exe_file_dir),
                        help="path to config file")
    parser.add_argument("-d", "--data_file", default="{}/cfg/voc.data".format(exe_file_dir),
                        help="path to data file")
    parser.add_argument("-t", "--thresh", type=float, default=.1,
                        help="remove detections with lower confidence")
    parser.add_argument("-w", "--weight", help="weight file",
                        default="{}/model_weight/yolo-fastest-xl_last.weights".format(exe_file_dir))


    return parser.parse_args()

def main():
    args = parser_args()
    global model
    model = YoloFastestModel(args.config_file, args.data_file, args.weight)
    
    
    rospy.init_node("subscirbeImage" , anonymous=True)
    img_subscriber_obj = ImageSubscriber("/pico_camera/color_image" , model = model)
    rospy.spin()
if __name__ == '__main__':
    main()
    