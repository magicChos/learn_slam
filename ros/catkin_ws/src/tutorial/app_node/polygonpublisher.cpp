/************************************************************************************************
@filename    :polygonpublisher.cpp
@brief       :发布多边形结点
@time        :2021/02/08 13:32:15
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/



#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点 节点名字
    ros::init(argc, argv, "polygonpublisher");

    // 节点句柄
    ros::NodeHandle nh;

    // // 发布消息 话题名字 队列大小
    // ros::Publisher pub = nh.advertise<geometry_msgs::PolygonStamped>("polygonpublisher", 1);

    // // 定义多边形对象
    // geometry_msgs::PolygonStamped myPolygon;

    // // 多边形数据
    // geometry_msgs::Point32 point;

    // point.x = -10;
    // point.y = -10;
    // point.z = 0;

    // myPolygon.polygon.points.push_back(point);

    // point.x = 10;
    // point.y = -10;
    // point.z = 0;

    // myPolygon.polygon.points.push_back(point);

    // point.x = 10;
    // point.y = 10;
    // point.z = 0;

    // myPolygon.polygon.points.push_back(point);

    // point.x = -10;
    // point.y = 10;
    // point.z = 0;

    // myPolygon.polygon.points.push_back(point);

    // // frame id
    // myPolygon.header.frame_id = "map";

    // // 消息发布频率
    // ros::Rate loop_rate(1);

    // while (ros::ok())
    // {
    //     // 广播
    //     pub.publish(myPolygon);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::Time begin = ros::Time::now();
    std::cout << begin << std::endl;
    return 0;
}
