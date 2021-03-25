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

void slamMapToMat(const nav_messages::FusionOccupancyGrid &map, cv::Mat &map_cv);

int64_t GetTimeStamp();
