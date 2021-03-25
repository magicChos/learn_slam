#include "subscriber/cloud_subscriber.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>

CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::parse_data(std::deque<CloudData> &deque_cloud_data)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    if (new_cloud_data_.size() > 0)
    {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud::ConstPtr &cloud_msg_ptr)
{
    std::lock_guard<std::mutex> guard(buff_mutex_);
    try
    {
        CloudData::CLOUD_PTR pcl_cloud_msg(new CloudData::CLOUD);

        sensor_msgs::PointCloud2 point_cloud;
        sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg_ptr, point_cloud);
        pcl::fromROSMsg(point_cloud, *pcl_cloud_msg);

        CloudData data;
        data.cloud_ptr = pcl_cloud_msg;
        data.timestamp = cloud_msg_ptr->header.stamp.toNSec() * 0.000001;
        new_cloud_data_.emplace_back(data);
       
    }
    catch (const std::string &s)
    {
        std::cout << "occur " << s << std::endl;
    }
}
