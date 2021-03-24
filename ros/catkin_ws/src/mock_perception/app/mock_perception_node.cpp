#include "tf_listener/tf_listener.hpp"
#include "subscriber/cloud_subscriber.h"
#include "subscriber/color_subscriber.h"
#include "subscriber/map_subscriber.h"
#include "utils/data_converter.hpp"
#include <memory>

int main(int argc, char **argv)
{
    std::cout << "hello mock perception system!" << std::endl;
    ros::init(argc, argv, "mock_perception_node");
    ros::NodeHandle nh;

    std::shared_ptr<ColorSubscriber> color_sub_ptr = std::make_shared<ColorSubscriber>(nh, "/pico_camera/color_image", 10000);
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/pico_camera/point_cloud", 10000);
    std::shared_ptr<MapSubscriber> map_sub_ptr = std::make_shared<MapSubscriber>(nh, "/map", 10000);
    std::shared_ptr<TFListener> robot_to_map_ptr = std::make_shared<TFListener>(nh , "map" , "base_footprint");
    std::deque<cv::Mat> color_data_buff;
    std::deque<CloudSubscriber::Cloud> cloud_data_buff;
    std::deque<nav_msgs::OccupancyGrid> map_data_buff;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        color_sub_ptr->parse_data(color_data_buff);
        cloud_sub_ptr->parse_data(cloud_data_buff);
        map_sub_ptr->parse_data(map_data_buff);

        


        while (cloud_data_buff.size() > 0 && color_data_buff.size() > 0 && map_data_buff.size() > 0)
        {
            cv::Mat color_img = color_data_buff.front();
            CloudSubscriber::Cloud point_cloud = cloud_data_buff.front();

            std::vector<Eigen::Vector3d> process_point_cloud;
            convert_point_cloud(point_cloud, process_point_cloud);

            // cv::Mat map;
            // getSingleLevelLocalMap(process_point_cloud, map);

            // cv::imshow("map", map);
            // cv::imshow("color" , color_img);
            color_data_buff.pop_front();
            cloud_data_buff.pop_front();

            cv::waitKey(0);
        }
    }

    return 1;
}