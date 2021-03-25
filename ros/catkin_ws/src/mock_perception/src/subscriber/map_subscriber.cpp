#include "subscriber/map_subscriber.h"
#include "utils/data_converter.hpp"

MapSubscriber::MapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &MapSubscriber::msg_callback, this);
}


bool MapSubscriber::parse_data(nav_messages::FusionOccupancyGrid &slam_map)
{
    if (m_receive_slammap)
    {
        slam_map = new_map_data;
    }

    return m_receive_slammap;
}

void MapSubscriber::parse_data(std::deque<nav_msgs::OccupancyGrid> &deque_map_data)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    if (new_map_buffer.size() > 0)
    {
        deque_map_data.insert(deque_map_data.end(), new_map_buffer.begin(), new_map_buffer.end());
        new_map_buffer.clear();
    }
}

void MapSubscriber::msg_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    if (msg == nullptr)
    {
        return;
    }

    if (msg->info.width == new_map_data.info.width && msg->info.height == new_map_data.info.height && new_map_data.data.size() == msg->data.size() || msg->data.size() == 0)
    {
        return;
    }

    convert_rosOccupancyGraid_FusionOccupancyGrid(msg , new_map_data);
    m_receive_slammap = true;
}
