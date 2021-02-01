#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <opencv2/opencv.hpp>

bool convert2localmap(cv::Mat &input_img, const std::string frame_id, const double resolution, nav_msgs::OccupancyGrid &cost_map)
{
    if (input_img.empty())
    {
        std::cerr << "input image is empty!\n";
        return false;
    }

    int height = input_img.rows;
    int width = input_img.cols;

    cost_map.header.frame_id = frame_id;
    cost_map.header.stamp = ros::Time::now();

    cost_map.info.resolution = resolution;
    cost_map.info.width = width;
    cost_map.info.height = height;

    int data[height * width] = {-1};
    for (int i = 0; i < height; ++i)
    {
        uchar *ptr = input_img.ptr<uchar>(i);
        for (int j = 0; j < width; ++j)
        {
            if (ptr[j] != 0)
            {
                data[i * width + j] = 100;
            }
        }
    }

    std::vector<signed char> a(data, data + width * height);
    cost_map.data = a;
    return true;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    nav_msgs::OccupancyGrid map;

    cv::Mat img = cv::imread("/home/han/data/project/learn_slam/ros/catkin_ws/src/grid/localmap.png", 0);

    convert2localmap(img , "grid" , 0.005 , map);

    while (ros::ok())
    {
        pub.publish(map);
    }

    ros::shutdown();
    return 0;
}
