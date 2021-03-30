
/************************************************************************************************
@filename    :debug_utils.h
@brief       :
@time        :2021/02/21 15:35:36
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "nav_messages/FusionOccupancyGrid.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/image_data.hpp"
#include "geometry_messages/Pose2D.hpp"

void slamMapToMat(const nav_messages::FusionOccupancyGrid &map, cv::Mat &map_cv);

int64_t GetTimeStamp();

Eigen::MatrixXd Mat2MatrixXd(const cv::Mat &R);

void printTimeStamp(const ImageData &image_data , const CloudData &cloud_data , const geometry_messages::Pose2D &robot_pose);

void printRobotPose(const geometry_messages::Pose2D &robot_pose);

