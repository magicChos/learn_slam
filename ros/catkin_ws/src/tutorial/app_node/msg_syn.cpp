#include "msg_syn.h"
#include <boost/thread/thread.hpp>
#include <iostream>
#include <ros/duration.h>

namespace climbing_robot
{

    MsgSynchronizer::MsgSynchronizer(ros::NodeHandle node)
    {
        message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "/pico_camera/color_image", 1);
        message_filters::Subscriber<sensor_msgs::PointCloud> cloud_sub(node, "/pico_camera/point_cloud", 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::Image> approximate_policy;
        message_filters::Synchronizer<approximate_policy> sync(approximate_policy(10), cloud_sub, image_sub);
        sync.registerCallback(boost::bind(&MsgSynchronizer::callback, this, _1, _2));
        syn_bag_path = "/home/han/project/learn_slam/ros/catkin_ws/src/tutorial/build/sync.bag";
        msg_syn_bag.open(syn_bag_path, rosbag::bagmode::Write);

        ros::spin();
    }

    void MsgSynchronizer::callback(const sensor_msgs::PointCloud::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_image)
    {
        ROS_INFO("pointcloud stamp value is: %f", ori_pointcloud->header.stamp.toSec());
        ROS_INFO("pose stamp value is: %f", ori_image->header.stamp.toSec());

        msg_syn_bag.write("/pointcloud", ori_pointcloud->header.stamp, *ori_pointcloud);
        msg_syn_bag.write("/image", ori_pointcloud->header.stamp, *ori_image);
    }

} //namespace climbing_robot