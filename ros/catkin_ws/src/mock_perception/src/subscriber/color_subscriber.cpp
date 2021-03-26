#ifndef __COLOR_SUBSCRIBER_H__
#define __COLOR_SUBSCRIBER_H__
#include "subscriber/color_subscriber.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <thread>

ColorSubscriber::ColorSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size)
    : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &ColorSubscriber::msg_callback, this);
}

void ColorSubscriber::parse_data(std::deque<ImageData> &deque_color_data)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    if (new_color_data_.size() > 0)
    {
        deque_color_data.insert(deque_color_data.end(), new_color_data_.begin(), new_color_data_.end());
        new_color_data_.clear();
    }
}

void ColorSubscriber::msg_callback(const sensor_msgs::Image::ConstPtr &color_msg_ptr)
{
    if (color_msg_ptr == nullptr)
    {
        return;
    }
    std::lock_guard<std::mutex> guard(buff_mutex_);
    try
    {
        std::cout << "sub color image" << std::endl;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(color_msg_ptr, sensor_msgs::image_encodings::BGR8);
        ImageData data;
        data.image = cv_ptr->image;
        data.timestamp = color_msg_ptr->header.stamp.toNSec() * 0.000001;
        std::cout << "color timestamp: " << data.timestamp << std::endl;
        new_color_data_.emplace_back(data);
    }
    catch (cv_bridge::Exception &e)
    {
        printf("Could not convert from '%s' to '32fc1'.", color_msg_ptr->encoding.c_str());
    }
}
#endif // __COLOR_SUBSCRIBER_H__