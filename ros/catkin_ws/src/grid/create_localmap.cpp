#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    nav_msgs::OccupancyGrid map;

    cv::Mat img = cv::imread("/home/han/data/project/learn_slam/ros/catkin_ws/src/grid/localmap.png" , 0);
    if (img.empty())
    {
        std::cerr << "read image failture!\n";
        return -1;
    }

    int height = img.rows;
    int width  = img.cols;


    map.header.frame_id = "grid";
    map.header.stamp = ros::Time::now();
    map.info.resolution = 0.005; // float32
    map.info.width = width;       // uint32
    map.info.height = height;      // uint32

    int p[map.info.width * map.info.height] = {-1}; // [0,100]

    for (int i = 0 ; i < height ; ++i)
    {   
        uchar *ptr = img.ptr<uchar>(i);
        for (int j = 0 ; j < width ; ++j)
        {
            if (ptr[j] != 0)
            {
                p[i * width + j] = 100;
            }
        }
    }


    // p[10] = 100;
    // std::vector<signed char> a(p, p + 400);
    std::vector<signed char> a(p , p + width * height);
    map.data = a;

    while (ros::ok())
    {
        pub.publish(map);
    }

    ros::shutdown();
    return 0;
}
