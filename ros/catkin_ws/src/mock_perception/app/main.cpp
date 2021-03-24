#include "system.h"
#include "module/perception_module.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_perception");
    ros::start();
    
    auto perception_module = std::make_shared<PerceptionModule>();
    auto system = std::make_shared<MockSystem>(perception_module);
    ros::spin();
    ros::shutdown();
}