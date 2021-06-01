#include "system.h"
#include "module/perception_module.h"
#include <ros/ros.h>
#include "common/log.h"
using namespace ace::common;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mock_perception");
    ros::start();
    Log::SetLevel(0);
    LogInfo("step into main function");

    auto perception_module = std::make_shared<PerceptionModule>();
    auto system = std::make_shared<MockSystem>(perception_module);
    ros::spin();
    ros::shutdown();
}