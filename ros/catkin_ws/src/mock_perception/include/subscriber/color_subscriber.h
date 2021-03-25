#pragma once

/************************************************************************************************
@filename    :color_subscriber.h
@brief       :
@time        :2021/03/01 16:39:44
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <mutex>
#include <opencv2/opencv.hpp>
#include "sensor_data/image_data.hpp"

class ColorSubscriber
{
public:
    ColorSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    ColorSubscriber() = default;

    void parse_data(std::deque<ImageData> &deque_color_data);

private:
    void msg_callback(const sensor_msgs::Image::ConstPtr &color_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<ImageData> new_color_data_;
    std::mutex buff_mutex_;
};
