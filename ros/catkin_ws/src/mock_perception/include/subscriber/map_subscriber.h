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
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include <opencv2/opencv.hpp>

class MapSubscriber
{
public:
    MapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    MapSubscriber() = default;

    void parse_data(std::deque<nav_msgs::OccupancyGrid> &deque_map_data);

private:
    void msg_callback(const nav_msgs::OccupancyGridConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OccupancyGrid> new_map_data_;
    std::mutex buff_mutex_;
};









