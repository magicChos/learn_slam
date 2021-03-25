// 模块基类

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <lcm_cpp/geometry_messages/Pose2D.hpp>
#include "lcm_cpp/nav_messages/FusionOccupancyGrid.hpp"

class BaseModule
{
public:
    BaseModule() = default;
    virtual ~BaseModule();

    virtual bool run() = 0;
    
    // return fusion map
    virtual cv::Mat run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose , const nav_messages::FusionOccupancyGrid &slam_map) = 0;
};