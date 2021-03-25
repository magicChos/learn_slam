/************************************************************************************************
@filename    :image_data.hpp
@brief       :
@time        :2021/03/25 10:00:38
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class ImageData
{
public:
    int64_t timestamp = 0;
    cv::Mat image;
};