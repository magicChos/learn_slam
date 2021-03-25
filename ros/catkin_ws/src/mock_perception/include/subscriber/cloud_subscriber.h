#pragma once

/************************************************************************************************
@filename    :
@brief       :
@time        :2021/03/01 16:39:44
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_data/cloud_data.hpp"

class CloudSubscriber
{
public:
    // using Cloud = pcl::PointCloud<pcl::PointXYZ>;

    CloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    CloudSubscriber() = default;

    void parse_data(std::deque<CloudData> &deque_cloud_data);

private:
    void msg_callback(const sensor_msgs::PointCloud::ConstPtr &cloud_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;
    std::mutex buff_mutex_;
};
