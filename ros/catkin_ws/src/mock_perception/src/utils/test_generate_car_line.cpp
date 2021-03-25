// /************************************************************************************************
// @filename    :test_generate_car_line.cpp
// @brief       :测试绘制车道线
// @time        :2020/11/22 00:16:29
// @author      :hscoder
// @versions    :1.0
// @email       :hscoder@163.com
// @usage       :
// ***********************************************************************************************/

// #include "draw_car_line.h"
// #include <iostream>
// #include <vector>
// #include <opencv2/opencv.hpp>

// using namespace ace::common;
// using namespace ace::perception;



// int main(int argc, char **argv)
// {
//     try
//     {
//         if (argc != 3)
//         {
//             std::cerr << "usage: exe imageName calibration_file" << std::endl;
//             return 0;
//         }
//         std::string yaml_file, imageName;
//         yaml_file = std::string(argv[2]);
//         imageName = std::string(argv[1]);

//         std::vector<float> cameraVec, distVec;
//         if (!readCameraMatrix(yaml_file, cameraVec, distVec))
//         {
//             std::cerr << "get camera intrinsic matrix failture!\n";
//             return 0;
//         }

//         cv::Mat cameraMat, distCoeff;
//         cameraMat = convertVector2Mat<float>(cameraVec, 1, 3);
//         distCoeff = convertVector2Mat<float>(distVec, 1, 5);

//         cv::Mat img = cv::imread(imageName);
//         if (!img.data)
//         {
//             std::cerr << "read image failture!\n";
//             return 0;
//         }

//         std::vector<cv::Point3f> car_left_line, car_right_line;
//         createCarline(car_left_line, car_right_line);
//         drawCarline(img, car_left_line, car_right_line, cameraMat, distCoeff);
//         cv::imshow("image", img);
//         cv::waitKey(0);
//     }
//     catch (const std::string &s)
//     {
//         std::cerr << s << std::endl;
//         return EXIT_FAILURE;
//     }

//     return 1;
// }
