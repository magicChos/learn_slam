#include "ace/perception/utils/draw_car_line.h"
#include "gtest/gtest.h"
#include "glog/logging.h"
#include "ace/perception/utils/file_system.hpp"
#include "ace/perception/utils/portability_fixes.hpp"
#include "ace/perception/utils/wildcard.hpp"
#include "ace/sensor/camera.h"

// using namespace ace::perception;

namespace ace
{
    namespace perception
    {

        void test_draw_car_line()
        {
            std::vector<cv::Point3f> car_left_line, car_right_line;
            createCarline(car_left_line, car_right_line);

            ace::sensor::Camera cam(0);
            cam.Initialize();
            std::cout << "-------------------------------------------------\n";
            cv::Mat cameraMat, distCoeff;
            cameraMat = cv::Mat::eye(3 , 3, CV_32F);
            distCoeff = cv::Mat::zeros(1 , 5 , CV_32F);

            cam.exportCameraIntrinsicParams(cameraMat, distCoeff);

            cv::Mat rgb;
            while (true)
            {
                if (!cam.ReadRGB(rgb))
                    continue;

                drawCarline(rgb, car_left_line, car_right_line, cameraMat, distCoeff);
                cv::imshow("rgb", rgb);
                if (cv::waitKey(25) == 'q')
                {
                    break;
                }
            }
        }

        TEST(DrawCarLine, DISABLED_drawCar)
        {
            // std::string current_dir = stlplus::folder_up(__FILE__);
            // std::string yaml_file;
            // std::string imageName;
            // yaml_file = current_dir + "../config/calibration.yaml";
            // imageName = current_dir + "../data_img/object.jpg";

            // std::vector<float> cameraVec, distVec;
            // if (!readCameraMatrix(yaml_file, cameraVec, distVec))
            // {
            //     LOG(ERROR) << "get camera intrinsic matrix failture!\n";
            // }

            // cv::Mat cameraMat, distCoeff;
            // cameraMat = convertVector2Mat<float>(cameraVec, 1, 3);
            // distCoeff = convertVector2Mat<float>(distVec, 1, 5);

            // std::cout << cameraMat << std::endl;

            // cv::Mat img = cv::imread(imageName);
            // if (!img.data)
            // {
            //     LOG(ERROR) << "read image failture!\n";
            // }

            // std::vector<cv::Point3f> car_left_line, car_right_line;
            // createCarline(car_left_line, car_right_line);
            // drawCarline(img, car_left_line, car_right_line, cameraMat, distCoeff);
            // cv::imshow("image", img);
            // cv::waitKey(0);


            test_draw_car_line();
        }

    } // namespace perception
} // namespace ace
