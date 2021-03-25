#include "module/perception_module.h"

PerceptionModule::PerceptionModule()
{
    std::cout << "step into perception module" << std::endl;
}

bool PerceptionModule::run()
{
    std::cout << "process perception module" << std::endl;
    return true;
}

bool PerceptionModule::GetLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, cv::Mat &local_map)
{
    // local_map = m_obstacle_detector->GenerateLocalMap(rgb_image , pointCloud)
    return true;
}

bool PerceptionModule::updateGlobalMap(const geometry_messages::Pose2D &robot_pose)
{
    return true;
}

bool PerceptionModule::fusion_process(cv::Mat &fusion_map)
{
    return true;
}

cv::Mat PerceptionModule::run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &slam_map)
{
    cv::Mat local_map;
    if (!GetLocalMap(rgb_image, pointCloud, local_map))
    {
        std::cout << "@test generate localmap failture" << std::endl;
        return cv::Mat();
    }
    updateGlobalMap(robot_pose);

    cv::Mat fusion_map;
    fusion_process(fusion_map);

    std::cout << "finished" << std::endl;

    return fusion_map;
}