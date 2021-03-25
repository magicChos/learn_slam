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
#include "lcm_cpp/nav_messages/FusionOccupancyGrid.hpp"

class MapSubscriber
{
public:
    MapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    MapSubscriber() = default;

    void parse_data(std::deque<nav_msgs::OccupancyGrid> &deque_map_data);

    // bool parse_data(nav_msgs::OccupancyGrid &slam_map);

    bool parse_data(nav_messages::FusionOccupancyGrid &slam_map);

private:
    void msg_callback(const nav_msgs::OccupancyGridConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OccupancyGrid> new_map_buffer;
    // nav_msgs::OccupancyGrid new_map_data;
    // nav_msgs::OccupancyGridConstPtr new_map_data;

    nav_messages::FusionOccupancyGrid new_map_data;

    std::mutex buff_mutex_;
    bool m_receive_slammap = false;
};













