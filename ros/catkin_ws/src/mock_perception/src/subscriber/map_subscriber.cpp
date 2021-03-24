#include "subscriber/map_subscriber.h"

MapSubscriber::MapSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &MapSubscriber::msg_callback, this);
}

void MapSubscriber::parse_data(std::deque<nav_msgs::OccupancyGrid> &deque_map_data)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    if (new_map_data_.size() > 0)
    {
        deque_map_data.insert(deque_map_data.end(), new_map_data_.begin(), new_map_data_.end());
    }
}

void MapSubscriber::msg_callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    try
    {
        new_map_data_.push_back(*msg);
    }
    catch (const std::string &s)
    {
        std::cout << "occur " << s << std::endl;
    }
}
