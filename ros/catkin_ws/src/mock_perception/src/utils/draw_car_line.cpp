#include "draw_car_line.h"
#include "readyaml.h"
#include <iostream>
#include <memory>
namespace ace
{
  namespace perception
  {
    bool readCameraMatrix(const std::string yaml_file,
                          cv::Mat &cameraMat,
                          cv::Mat &distCoeff)
    {
      std::shared_ptr<YamlReader> yaml_reader_obj = std::make_shared<YamlReader>(yaml_file);
      yaml_reader_obj->getNodeMatrix("camera", cameraMat);
      yaml_reader_obj->getNodeMatrix("dist", distCoeff);

      return true;
    }

    void project_point_to_pixel(cv::Point2i &project_xy, const cv::Mat &cameraMat,
                                const cv::Mat &distCoeff,
                                const cv::Point3f &point)
    {
      double x = point.x / point.z;
      double y = point.y / point.z;

      double r2 = x * x + y * y;
      double k1 = distCoeff.at<float>(0, 0);
      double k2 = distCoeff.at<float>(0, 1);
      // double p1 = distCoeff.at<float>(0, 2);
      // double p2 = distCoeff.at<float>(0, 3);
      // double k3 = distCoeff.at<float>(0, 4);

      double f = 1 + k1 * r2 + k2 * r2 * r2;
      x *= f;
      y *= f;

      double fx = cameraMat.at<float>(0, 0);
      double fy = cameraMat.at<float>(1, 1);
      double cx = cameraMat.at<float>(0, 2);
      double cy = cameraMat.at<float>(1, 2);

      project_xy.x = static_cast<int>(x * fx + cx);
      project_xy.y = static_cast<int>(y * fy + cy);
    }

    void createCarline(std::vector<cv::Point3f> &left_car_line,
                       std::vector<cv::Point3f> &right_car_line)
    {
      // left_car_line.emplace_back(cv::Point3f(0.145, 0.1, 0.1));
      // right_car_line.emplace_back(cv::Point3f(-0.145, 0.1, 0.1));
      left_car_line.emplace_back(cv::Point3f(0.144, 0.1, 0.1));
      right_car_line.emplace_back(cv::Point3f(-0.206, 0.1, 0.1));

      for (int i = 1; i < 5; ++i)
      {
        left_car_line.emplace_back(cv::Point3f(0.144, 0.1, 0.5 * i));
        right_car_line.emplace_back(cv::Point3f(-0.206, 0.1, 0.5 * i));
      }
    }

    void drawCarline(cv::Mat &img_src,
                     const std::vector<cv::Point3f> &left_car_line,
                     const std::vector<cv::Point3f> &right_car_line,
                     const cv::Mat &cameraMat, const cv::Mat &distCoef)
    {
      std::vector<cv::Point2i> left_line_pixel, right_line_pixel;

      // left line
      for (auto left_pt : left_car_line)
      {
        cv::Point2i tmp;
        project_point_to_pixel(tmp, cameraMat, distCoef, left_pt);
        left_line_pixel.emplace_back(tmp);
      }

      // right line
      for (auto right_pt : right_car_line)
      {
        cv::Point2i tmp;
        project_point_to_pixel(tmp, cameraMat, distCoef, right_pt);
        right_line_pixel.emplace_back(tmp);
      }

      cv::Point2i start_pt = left_line_pixel[0];
      cv::Point2i end_pt = left_line_pixel[left_line_pixel.size() - 1];
      cv::line(img_src, start_pt, end_pt, cv::Scalar(0, 0, 255), 2);

      start_pt = right_line_pixel[0];
      end_pt = right_line_pixel[right_line_pixel.size() - 1];
      cv::line(img_src, start_pt, end_pt, cv::Scalar(0, 0, 255), 2);

      for (size_t i = 0; i < left_line_pixel.size(); ++i)
      {
        cv::Point2i left_pt = left_line_pixel[i];
        cv::Point2i right_pt = right_line_pixel[i];
        cv::line(img_src, left_pt, right_pt, cv::Scalar(0, 0, 255), 1);
      }
    }
  } // namespace perception
} // namespace ace
