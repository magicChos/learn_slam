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

import cv2
from subscriber.subscriber_image import ImageSubscriber
from publisher.publisher_image import ImagePublisher
from carline.draw_car_line import DrawCarLine
from utils.yaml_reader import read_yaml_cv, read_yaml
from utils.cloud_filter import custom_filter
import argparse
import os
import rospy
import sys
from collections import deque
# import open3d as o3d
# from vis_3d import create_pointcloud_from_depth_and_rgb_cv, create_pointcloud_from_depth_and_rgb_roi
# from vis_3d import createMap
sys.path.append("yoloapi")
from make_predict import YoloFastestModel
exe_file_dir = os.getcwd()


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
    parser.add_argument("-i", "--intrisic", help='camera intrinsic matrix',
                        default="{}/cfg/camera_param.yaml".format(exe_file_dir))
    parser.add_argument("-e", "--extrinsic", help="camera extrinsic matrix",
                        default="{}/cfg/calibration.yaml".format(exe_file_dir))
    parser.add_argument("-p", "--predict", help="inference flag", default=True)

    return parser.parse_args()


def main():
    args = parser_args()
    camera_matrix = read_yaml_cv(
        args.intrisic, ["camera_matrix", "dist_matrix", "extrinsic_matrix"])
    extrinsic_matrix = read_yaml(args.extrinsic)
    camera_to_base_matrix = extrinsic_matrix['camera_to_base']

    model = None
    if args.predict:
        model = YoloFastestModel(args.config_file, args.data_file, args.weight)

    draw_car_line_object = DrawCarLine(camera_matrix)
    # axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[
    #                                                              0, 0, 0])
    # axis_pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0],
    #                     [0, 0, 1, 0], [0, 0, 0, 1]])

    rospy.init_node("subscirbeImage", anonymous=True)
    img_subscriber_obj = ImageSubscriber(
        "/pico_camera/color_image", buffer_size=100, show_result=True , model=model , draw_car_line_obj=draw_car_line_object)
    # depth_subscriber_obj = ImageSubscriber(
    #     "/pico_camera/depth_image", buffer_size=100, depth_flag=True, show_result=False)

    # rate = rospy.Rate(10)
    # while True:
    #     image_q = img_subscriber_obj.parse_data()
    #     depth_q = depth_subscriber_obj.parse_data()

    #     while image_q and depth_q:
    #         image_data = image_q[0]
    #         depth_data = depth_q[0]
    #         d_time = image_data.time_info() - depth_data.time_info()

    #         if d_time < -0.05:
    #             image_q.popleft()
    #         elif d_time > 0.05:
    #             depth_q.popleft()
    #         else:
    #             image_q.popleft()
    #             depth_q.popleft()

    #             image_raw = image_data.image_info()
    #             depth_raw = depth_data.image_info()
    #             color_img_RGB = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)

    #             draw_geometry = []

    #             if model is not None:
    #                 image_raw, detections = model.predict_cv(image_raw)
    #                 roi_bboxes = []
    #                 roi_pcds = create_pointcloud_from_depth_and_rgb_roi(
    #                     depth_raw, camera_matrix['camera_matrix'], detections, flatten=True)
    #                 for roi_pcd in roi_pcds:
    #                     roi_cloud = o3d.geometry.PointCloud()
    #                     roi_cloud.points = o3d.utility.Vector3dVector(roi_pcd)
    #                     roi_cloud.transform(camera_to_base_matrix)
    #                     roi_cloud.transform(
    #                         [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    #                     print("before filter: ", roi_cloud)
    #                     indices = custom_filter(
    #                         roi_cloud, [-0.3, 0.3], [0.3, 2.0], [0.03, 0.5])
    #                     inlier_cloud = roi_cloud.select_by_index(indices)

    #                     inlier_cloud, _ = inlier_cloud.remove_radius_outlier(
    #                         10, 0.01)

    #                     print("after filter: ", inlier_cloud)

    #                     alignbbox = inlier_cloud.get_axis_aligned_bounding_box()
    #                     alignbbox.color = (0, 1, 0)
    #                     draw_geometry.append(alignbbox)

    #                     max_bbox = alignbbox.get_max_bound().tolist()[:2]
    #                     min_bbox = alignbbox.get_min_bound().tolist()[:2]

    #                     roi_bboxes.append((min_bbox, max_bbox))


    #             draw_car_line_object.drawline(image_raw)
    #             tmp = create_pointcloud_from_depth_and_rgb_cv(
    #                 depth_raw, color_img_RGB, camera_matrix['camera_matrix'], camera_to_base_matrix)
    #             map_img = createMap(tmp, res=0.005, bboxes=roi_bboxes)
    #             cv2.imshow("map", map_img)
    #             cv2.imshow("image", image_raw)
    #             cv2.waitKey(25)
    #             o3d.visualization.draw_geometries(draw_geometry , width=640, height=480)
    #             roi_bboxes = []

    rospy.spin()


if __name__ == '__main__':
    main()
