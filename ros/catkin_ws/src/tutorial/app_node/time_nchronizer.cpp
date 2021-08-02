/************************************************************************************************
@filename    :time_nchronizer.cpp
@brief       :image和pointcloud同步
@time        :2021/08/02 01:16:36
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/




#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;

class mySynchronizer
{
public:
    ros::Publisher pubPointCloud;
    ros::Publisher pubImage;

    mySynchronizer();
    ~mySynchronizer(){};

    void callback(const sensor_msgs::PointCloud::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_imu);
};

void mySynchronizer::callback(const sensor_msgs::PointCloud::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_image)
{
    cout << "*******************" << endl;
    sensor_msgs::PointCloud syn_pointcloud = *ori_pointcloud;
    sensor_msgs::Image syn_image = *ori_image;

    ROS_INFO("pointcloud stamp value is: %f", syn_pointcloud.header.stamp.toSec());
    ROS_INFO("image stamp value is: %f", syn_image.header.stamp.toSec());

    pubPointCloud.publish(syn_pointcloud);
    pubImage.publish(syn_image);
}

mySynchronizer::mySynchronizer()
{
    ros::NodeHandle nh;
    pubPointCloud = nh.advertise<sensor_msgs::PointCloud>("/Syn/imu/data", 1);
    pubImage = nh.advertise<sensor_msgs::Image>("/Syn/velodyne_points", 1);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/pico_camera/color_image", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> velodyne_sub(nh, "/pico_camera/point_cloud", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, image_sub);
    sync.registerCallback(boost::bind(&mySynchronizer::callback, this, _1, _2));

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_synchronizer");
    ROS_INFO("\033[1;32m---->\033[0m Sync msgs node Started.");

    mySynchronizer wode;
    return 0;
}