#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/bag_player.h>
#include <vector>

using namespace std;
typedef pcl::PointXYZRGB PointType;
vector<sensor_msgs::PointCloud2> lidar_datas;

void loadPointcloudFromROSBag(const string &bag_path)
{
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    rosbag::Bag bag;
    try
    {
        bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch (rosbag::BagException e)
    {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    // vector<string> types;
    // types.push_back(string("sensor_msgs/PointCloud2")); // message title
    // rosbag::View view(bag, rosbag::TypeQuery(types));

    vector<string> topics;
    topics.push_back(string("/Syn/lidar"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &m : view)
    {
        sensor_msgs::PointCloud2 livoxCloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message type
        lidar_datas.push_back(livoxCloud);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "merge_cloud");
    ros::NodeHandle n;

    ROS_INFO("\033[1;32m---->\033[0m merge_cloud node Started. ");

    std::string input_bag_path = "/home/han/8-9/2021-08-09-15-47-07.bag";
    loadPointcloudFromROSBag(input_bag_path);
    uint64_t num = 0;
    pcl::PointCloud<PointType>::Ptr result_cloud(new pcl::PointCloud<PointType>);
    while (n.ok())
    {
        ros::spinOnce();
        if (num < lidar_datas.size())
        {
            std::cout << "num: " << num << std::endl;
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

            // pcl_conversions::toPCL(lidar_datas[num].header ,cloud->header);
            
            pcl::fromROSMsg<pcl::PointXYZRGB>(lidar_datas[num], *cloud);
            std::cout << "cloud size: " << cloud->points.size() << std::endl;

            *result_cloud += *cloud;
            std::cout << "result cloud size: " << result_cloud->points.size() << std::endl;
            ++num;
        }
        else
        {
            break;
        }
    }

    pcl::PCDWriter writer;
    writer.write("/home/han/project/learn_slam/ros/catkin_ws/src/tutorial/build/merge_cloud.pcd", *result_cloud);

    return 0;
}