/************************************************************************************************
@filename    :draw_car_line.h
@brief       :生成车道线
@time        :2020/11/21 23:23:56
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/
#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


namespace ace
{
    namespace perception
    {
        template <typename _Tp>
        cv::Mat convertVector2Mat(std::vector<_Tp> v, int channels, int rows)
        {
            cv::Mat mat = cv::Mat(v); //将vector变成单列的mat
            cv::Mat dest =
                mat.reshape(channels, rows).clone(); // PS：必须clone()一份，否则返回出错
            return dest;
        }

        bool readCameraMatrix(const std::string yaml_file,
                              cv::Mat &cameraMat,
                              cv::Mat &distCoeff);

        void project_point_to_pixel(cv::Point2i &project_xy,
                                    const cv::Mat &cameraMat,
                                    const cv::Mat &distCoeff,
                                    const cv::Point3f &point);

        void createCarline(std::vector<cv::Point3f> &left_car_line,
                           std::vector<cv::Point3f> &right_car_line);

        void drawCarline(cv::Mat &img_src,
                         const std::vector<cv::Point3f> &left_car_line,
                         const std::vector<cv::Point3f> &right_car_line,
                         const cv::Mat &cameraMat, const cv::Mat &distCoef);
    } // namespace perception
} // namespace ace
