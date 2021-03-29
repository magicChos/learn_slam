

#include "obstacle_detection/obstacle_detection.h"
#include "common/log.h"
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace ace
{
  namespace perception
  {
    static const char *sric_names[] = {"background", "bottle", "shoes", "wire", "trash can", "clothes"};

    ObstacleDetector::ObstacleDetector(ace::sensor::CameraInterface *camera,
                                       ObstacleDetectOption &option)
        : m_camera(camera), option(option)
    {
      m_timer = std::make_shared<Timer>();
      cameraParamsInit();
    };

    bool ObstacleDetector::GenerateLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, cv::Mat &localmap)
    {
      bool success = getSingleLevelLocalMap(pointCloud, localmap);
      if (!success)
      {
        LogError("generate local map failture");
        return false;
      }

      if (option.detectObjects)
      {
        return detectObjects(localmap, rgb_image);
      }

      return true;
    }

    bool ObjectDetectionWrapper::Detect(cv::Mat image,
                                        std::vector<OBJECT> &objects)
    {
      return sric_detection_process(image, objects) >= 0;
    }

    bool ObstacleDetector::detectObjects(cv::Mat &map, const cv::Mat rgb)
    {
      if (option.visualize)
        cv::cvtColor(map, map, cv::COLOR_GRAY2BGR);

      std::vector<OBJECT> objects;

      m_timer->Tic();
      if (!ObjectDetectionWrapper::get().Detect(rgb, objects))
      {
        LogError("Object detection failed");
        return false;
      }
      m_timer->Toc();
      double cost_time = m_timer->Elasped();
      // std::cout << "@test inference cost time: " << cost_time << std::endl;

      if (option.debug)
      {
        cv::Mat temp_rgb = rgb.clone();
        drawDetects(objects, temp_rgb);
        cv::imshow("detect", temp_rgb);
      }

      Eigen::Matrix3f &C = m_C;
      int rgbOriginV = m_rgbOrigin3D.x();
      float depth = m_rgbOrigin3D.z();

      for (const auto &object : objects)
      {
        cv::Vec3b color(0, 0, 0);
        color[object.label % 3] = 255;

        Eigen::Vector3d pointLeft{
            (object.rect.x + object.rect.width - C(0, 0)) / C(0, 2) * depth,
            (object.rect.y - C(1, 1)) / C(1, 2) * depth, depth};
        Eigen::Vector3d pointRight{(object.rect.x - C(0, 0)) / C(0, 2) * depth,
                                   (object.rect.y - C(1, 1)) / C(1, 2) * depth,
                                   depth};

        int leftV = int(pointLeft.x() / option.resolution) + (option.width / 2);
        // leftU = option.height - int(pointLeft.z() / option.resolution);
        int rightV = int(pointRight.x() / option.resolution) + (option.width / 2);
        // rightU = option.height - int(pointRight.z() / option.resolution);

        std::vector<cv::Point2i> objectPoints;
        std::vector<int> objectUs;

        for (int u = option.height - 1; u >= 0; --u)
        {
          int v0 = rgbOriginV +
                   (leftV - rgbOriginV) * (option.height - u) / option.height;
          int v1 = rgbOriginV +
                   (rightV - rgbOriginV) * (option.height - u) / option.height;
          if (v0 > v1)
          {
            std::swap(v0, v1);
          }
          v0 = std::max(v0, 0);
          v1 = std::min(v1, option.width - 1);
          for (int v = v0; v <= v1; ++v)
          {
            if (option.visualize)
            {
              auto &c = map.at<cv::Vec3b>(u, v);
              if (c[0] == 255 || c[1] == 255 || c[2] == 255)
              {
                // c = color;
                objectPoints.push_back({u, v});
                objectUs.push_back(u);
              }
            }
            else
            {
              uint8_t &c = map.at<uint8_t>(u, v);
              if (c == 255)
              {
                // c = object.label % 255;
                objectPoints.push_back({u, v});
                objectUs.push_back(u);
              }
            }
          }
        }
        if (objectUs.size() == 0)
          continue;

        std::sort(objectUs.rbegin(), objectUs.rend());
        int startU = objectUs.front(), endU = objectUs.back();
        {
          size_t i;
          for (i = 1; i < objectUs.size() - 1; ++i)
          {
            if (startU - objectUs[i] < 5)
              break;
            startU = objectUs[i];
          }
          endU = objectUs[i];
          for (; i < objectUs.size(); ++i)
          {
            if (endU - objectUs[i] >= 5)
              break;
            endU = objectUs[i];
          }
        }

        for (const auto &point : objectPoints)
        {
          if (point.x > startU || point.x < endU)
            continue;

          if (option.visualize)
          {
            map.at<cv::Vec3b>(point.x, point.y) = color;
          }
          else
          {
            map.at<uint8_t>(point.x, point.y) = object.label + 200;
          }
        }

        if (option.visualize)
        {
          // cv::line(map, cv::Point(leftV, leftU), cv::Point(rgbOriginV,
          // rgbOriginU),
          //          color);
          // cv::line(map, cv::Point(rightV, rightU),
          //          cv::Point(rgbOriginV, rgbOriginU), color);
        }
      }

      return true;
    }

    bool ObstacleDetector::GetLocalMap(cv::Mat &map)
    {
      if (m_camera == nullptr)
      {
        LogError("camera init failture");
        return false;
      }

      cv::Mat rgb;
      std::vector<Eigen::Vector3d> pointCloud;
      if (!m_camera->ReadFrame(rgb, pointCloud))
      {
        LogError("ObstacleDetector reads camera frame failed");
        return false;
      }

      // std::vector<Eigen::Vector3d> filter_pointCloud;
      // // 阈值待测试寻求最佳阈值
      // radisuFilter(pointCloud, filter_pointCloud, 50, 10);

      bool success = getSingleLevelLocalMap(pointCloud, map);
      if (!success)
      {
        LogError("generate local map failture");
        return false;
      }

      if (option.detectObjects)
      {
        return detectObjects(map, rgb);
      }

      return true;
    }

    bool ObstacleDetector::GetLocalMap(uint8_t *map)
    {
      cv::Mat mat;
      bool success = GetLocalMap(mat);
      if (!success)
        return false;
      if (mat.type() != 0)
        return false;

      if (map == nullptr)
      {
        map = new uint8_t[mat.rows * mat.cols];
      }

      memcpy((uint8_t *)mat.data, map, mat.rows * mat.cols);
      return true;
    }

    bool ObstacleDetector::getSingleLevelLocalMap(
        const std::vector<Eigen::Vector3d> &pointCloud, cv::Mat &map)
    {
      if (pointCloud.size() < 10)
      {
        return false;
      }

      const double resolution = option.resolution; // lenght of 1px in mm
      const double resolution_inv = 1 / resolution;
      const int width = option.width;   // horizontal (left/right) resolution
      const int height = option.height; // vertical (forward) resolution
      const double measureDistance_inv = 1 / 100.0;

      map = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));

      for (const auto &p : pointCloud)
      {
        if (p.z() >= option.lidarTop || p.z() <= option.baseBottom)
        {
          continue;
        }

        int leftV = int(p.x() * resolution_inv) + (width / 2),
            rightU = height - int(p.y() * resolution_inv);

        if (rightU < 0 || rightU >= height || leftV < 0 || leftV >= width)
        {
          continue;
        }

        auto &v = map.at<float>(rightU, leftV);
        v += (p.y() * measureDistance_inv) * m_blockProb;
      }

      const double threshold = 0.5;

      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

      cv::threshold(map, map, threshold, 1.0, cv::THRESH_TOZERO);
      map *= 255;
      map.convertTo(map, CV_8UC1);
      contourFilter(map);

      cv::morphologyEx(map, map, cv::MORPH_CLOSE, kernel);
      cv::Mat mapMask = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
      cv::Mat unknownMask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
      const int step = 400;

      int boundary = map.cols / 2 / std::atan(option.hfov / 2);
      for (int i = boundary; i < 2 * map.rows + map.cols - 2 - boundary; ++i)
      {
        int u, v;
        int uo = map.rows - 1, vo = map.cols / 2;
        if (i < map.rows)
        {
          u = map.rows - 1 - i;
          v = 0;
        }
        else if (i < map.rows + map.cols - 1)
        {
          u = 0;
          v = i - map.rows + 1;
        }
        else
        {
          u = i - map.rows - map.cols + 1;
          v = map.cols - 1;
        }
        double du = double(u - uo) / step, dv = double(v - vo) / step;

        bool foundBlock = false;

        for (int s = 1; s <= step; s++)
        {
          int uc = du * s + uo, vc = dv * s + vo;
          auto &v = map.at<uchar>(uc, vc);
          if (foundBlock)
            continue;
          double dist = std::sqrt((map.rows - 1 - uc) * (map.rows - 1 - uc) +
                                  (map.cols / 2 - vc) * (map.cols / 2 - vc)) *
                        option.resolution;
          if (dist >= option.minimumVisibleDistance &&
              dist <= option.maximumVisibleDistance)
          {
            unknownMask.at<uint8_t>(uc, vc) = 0;
          }
          if (v > 0)
          {
            if (dist <= option.minimumVisibleDistance)
            {
              unknownMask.at<uint8_t>(uc, vc) = 0;
            }
            mapMask.at<uint8_t>(uc, vc) = 255;
            foundBlock = true;
            continue;
          }
        }
      }

      for (int u = 0; u < map.rows; ++u)
      {
        for (int v = 0; v < map.cols; ++v)
        {
          if (mapMask.at<uint8_t>(u, v) == 0)
          {
            map.at<uchar>(u, v) = 0;
          }
        }
      }
      cv::morphologyEx(map, map, cv::MORPH_CLOSE, kernel);

      for (int u = 0; u < map.rows; ++u)
      {
        for (int v = 0; v < map.cols; ++v)
        {
          if (unknownMask.at<uint8_t>(u, v) == 255)
          {
            map.at<uchar>(u, v) = 127;
          }
        }
      }

      return true;
    }

    bool ObstacleDetector::drawDetects(const std::vector<OBJECT> &dets, cv::Mat &input_img)
    {
      if (dets.empty())
      {
        std::cout << "@test no detect result" << std::endl;
        return false;
      }

      // std::cout << "@test ----------------------------------------------" << std::endl;
      for (const auto &object : dets)
      {
        cv::rectangle(input_img, object.rect, cv::Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f%%", sric_names[object.label], object.prob * 100);
        // std::cout << "det obj label: " << sric_names[object.label] << std::endl;

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = object.rect.x;
        int y = object.rect.y - label_size.height - baseLine;
        if (y < 0)
          y = 0;
        if (x + label_size.width > input_img.cols)
          x = input_img.cols - label_size.width;

        cv::rectangle(
            input_img,
            cv::Rect(cv::Point(x, y),
                     cv::Size(label_size.width, label_size.height + baseLine)),
            cv::Scalar(255, 255, 255), -1);

        cv::putText(input_img, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
      }
      // std::cout << "@test -------------------------------------------end" << std::endl;

      return true;
    }

    bool ObstacleDetector::radisuFilter(const std::vector<Eigen::Vector3d> &input_pointCloud, std::vector<Eigen::Vector3d> &output_pointCloud, int size, float radius)
    {
      // open3d::geometry::PointCloud pointcloud_obj;
      // pointcloud_obj.points_.resize(input_pointCloud.size());
      // pointcloud_obj.points_ = input_pointCloud;
      // std::cout << "@test filter before: " << input_pointCloud.size() << std::endl;
      // auto filter = pointcloud_obj.RemoveRadiusOutliers(size, radius);

      // std::cout << "@test filter after: " << std::get<0>(filter)->points_.size() << std::endl;

      // output_pointCloud = std::get<0>(filter)->points_;
      return true;
    }

    void ObstacleDetector::cameraParamsInit()
    {
      m_camera->GetCalibration(m_C);
      Eigen::Matrix3f R;
      Eigen::Vector3f T;

      m_camera->GetExtrinsic(R, T);

      Eigen::Vector3f rgbOrigin;
      rgbOrigin << 0, 0, 0;
      rgbOrigin = R.inverse() * (rgbOrigin - T);

      int rgbOriginV = int(rgbOrigin.x() / option.resolution) + (option.width / 2),
          rgbOriginU = option.height - int(rgbOrigin.z() / option.resolution);

      float depth = option.height * option.resolution;
      m_rgbOrigin3D << rgbOriginV, rgbOriginU, depth;

      const double lidarTop = option.lidarTop;
      const double baseBottom = option.baseBottom;
      const double measureDensity = 2.0;

      m_blockProb = 1 / (option.resolution / measureDensity *
                         (lidarTop - baseBottom) / measureDensity);
    }

    int ObstacleDetector::contourFilter(cv::Mat &input_map)
    {
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> g_vHierarchy;

      cv::findContours(input_map, contours, g_vHierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
      for (size_t i = 0; i < contours.size(); ++i)
      {
        if (contours[i].size() < 10)
        {
          for (size_t j = 0; j < contours[i].size(); ++j)
          {
            input_map.at<uchar>(contours[i][j]) = 0;
          }
        }
      }
      return contours.size();
    }

    bool ObstacleDetector::getThreeLevelLocalMap(
        const std::vector<Eigen::Vector3d> &pointCloud, cv::Mat &map)
    {

      if (pointCloud.size() < 100)
        return false;

      // TODO: put the params into config files
      const double resolution = option.resolution; // lenght of 1px in mm
      const int width = option.width;              // horizontal (left/right) resolution
      const int height = option.height;            // vertical (forward) resolution

      // height in camera's coordinate
      const double lidarTop = option.lidarTop;
      const double baseTop = option.baseTop;
      const double armTop = option.armTop;
      const double armBottom = option.armBottom;
      const double baseBottom = option.baseBottom;

      const double measureDistance = 100.0;
      const double measureDensity = 2.0;

      const double blockArmProb =
          1 / (resolution / measureDensity * (armTop - armBottom) / measureDensity);
      const double blockBaseProb = 1 / (resolution / measureDensity *
                                        (baseTop - baseBottom) / measureDensity);
      const double blockLidarProb =
          1 / (resolution / measureDensity * (lidarTop - baseTop) / measureDensity);

      cv::Mat blockArm = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
      cv::Mat blockBase = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
      cv::Mat blockLidar = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));

      for (const auto &p : pointCloud)
      {

        if (p.z() >= lidarTop || p.z() <= baseBottom ||
            p.y() < std::numeric_limits<double>::epsilon())
        {
          continue;
        }

        int leftV = int(p.x() / resolution) + (width / 2),
            rightU = height - int(p.y() / resolution);

        if (rightU < 0 || rightU >= height || leftV < 0 || leftV >= width)
        {
          continue;
        }

        if (p.z() <= armTop && p.z() >= armBottom)
        {
          auto &v = blockArm.at<float>(rightU, leftV);
          v += (p.y() / measureDistance) * blockArmProb;
        }
        if (p.z() <= baseTop && p.z() >= baseBottom)
        {
          auto &v = blockBase.at<float>(rightU, leftV);
          v += (p.y() / measureDistance) * blockBaseProb;
        }
        if (p.z() >= baseTop && p.z() < lidarTop)
        {
          auto &v = blockLidar.at<float>(rightU, leftV);
          v += (p.y() / measureDistance) * blockLidarProb;
        }
      }

      cv::Mat *maps[] = {&blockBase, &blockArm, &blockLidar};

      const double threshold = 0.5;

      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

      for (auto map : maps)
      {
        cv::threshold(*map, *map, threshold, 1.0, cv::THRESH_TOZERO);
        *map *= 255;
        map->convertTo(*map, CV_8UC1);
        cv::morphologyEx(*map, *map, cv::MORPH_CLOSE, kernel);
      }

      map = cv::Mat(height, width, CV_8UC3);
      for (int u = 0; u < height; ++u)
      {
        for (int v = 0; v < width; ++v)
        {
          map.at<cv::Vec3b>(u, v) =
              cv::Vec3b(blockBase.at<uchar>(u, v), blockLidar.at<uchar>(u, v),
                        blockArm.at<uchar>(u, v));
        }
      }

      cv::Mat mapMask = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
      const int step = 400;

      for (int i = 0; i < 2 * map.rows + map.cols - 2; ++i)
      {
        int u, v;
        int uo = map.rows - 1, vo = map.cols / 2;
        if (i < map.rows)
        {
          u = map.rows - 1 - i;
          v = 0;
        }
        else if (i < map.rows + map.cols - 1)
        {
          u = 0;
          v = i - map.rows + 1;
        }
        else
        {
          u = i - map.rows - map.cols + 1;
          v = map.cols - 1;
        }
        double du = double(u - uo) / step, dv = double(v - vo) / step;

        bool foundBlockArm = false, foundBlockBase = false, foundBlockLidar = false;

        for (int s = 1; s <= step; s++)
        {
          int uc = du * s + uo, vc = dv * s + vo;
          auto &v = map.at<cv::Vec3b>(uc, vc);
          if (foundBlockArm)
            continue;
          if (v.val[2] > 0)
          {
            mapMask.at<uint8_t>(uc, vc) = 255;
            v.val[0] = v.val[1] = 0;
            foundBlockArm = true;
            continue;
          }
          if (foundBlockBase)
            continue;
          if (v.val[0] > 0)
          {
            v.val[1] = 0;
            mapMask.at<uint8_t>(uc, vc) = 255;
            foundBlockBase = true;
            continue;
          }
          if (foundBlockLidar)
            continue;
          if (v.val[1] > 0)
          {
            mapMask.at<uint8_t>(uc, vc) = 255;
            foundBlockLidar = true;
            continue;
          }
        }
      }

      for (int u = 0; u < map.rows; ++u)
      {
        for (int v = 0; v < map.cols; ++v)
        {
          if (mapMask.at<uint8_t>(u, v) == 0)
          {
            map.at<cv::Vec3b>(u, v) = cv::Vec3b(0, 0, 0);
          }
        }
      }
      cv::morphologyEx(map, map, cv::MORPH_CLOSE, kernel);

      return true;
    } // namespace perception
  }   // namespace perception
} // namespace ace
