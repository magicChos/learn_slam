#include "subscriber/cloud_subscriber.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::parse_data(std::deque<Cloud> &deque_cloud_data)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    if (new_cloud_data_.size() > 0)
    {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    }
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud::ConstPtr &cloud_msg_ptr)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    try
    {
        Cloud::Ptr pcl_cloud_msg(new Cloud);
        sensor_msgs::PointCloud2 point_cloud;
        sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg_ptr, point_cloud);
        pcl::fromROSMsg(point_cloud, *pcl_cloud_msg);
        new_cloud_data_.push_back(*pcl_cloud_msg);
    }
    catch (const std::string &s)
    {
        std::cout << "occur " << s << std::endl;
    }
}
