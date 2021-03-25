
#include "ace/sensor/camera.h"

#include <Eigen/Core>
#include <fstream>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <vector>

using namespace ace::sensor;

void test_read_rgb()
{
  Camera cam(0);
  cam.Initialize();

  cv::Mat rgb;
  while (true)
  {
    if (!cam.ReadRGB(rgb))
      continue;

    std::cout << "rgb   resolution: (" << rgb.rows << "," << rgb.cols << ")\n";
    cv::imshow("rgb", rgb);
    if (cv::waitKey(25) == 'q')
    {
      break;
    }
  }
}

void test_read_depth()
{
  Camera cam(0);
  cam.Initialize();

  cv::Mat depth;
  int count = 0;
  while (true)
  {
    if (!cam.ReadDepth(depth))
      continue;
    cv::imshow("depth", depth);
    char key = cv::waitKey(50);
    if (key == 'q')
    {
      break;
    }
    else if (key == 's')
    {
      std::cout << "***********************************\n";
      std::string save_depth_name = "depth_" + std::to_string(count) + ".png";
      count++;
      cv::imwrite(save_depth_name, depth);
    }
  }
}

void test_read_rgb_and_depth()
{
  Camera cam(0);
  cam.Initialize();

  cv::Mat depth, rgb;
  int count = 0;
  while (true)
  {
    std::cout << "----------------------------------------------\n";
    if (!cam.ReadRGBandDepth(rgb, depth))
      continue;

    std::cout << "depth resolution: (" << depth.rows << "," << depth.cols
              << ")\n";
    std::cout << "rgb   resolution: (" << rgb.rows << "," << rgb.cols << ")\n";
    cv::imshow("depth", depth);
    cv::imshow("rgb", rgb);

    char key = cv::waitKey(50);
    if (key == 'q')
    {
      break;
    }
    else if (key == 's')
    {
      std::cout << "***********************************\n";
      std::string save_depth_name = "depth_" + std::to_string(count) + ".png";
      std::string save_rgb_name = "rgb_" + std::to_string(count) + ".png";

      std::cout << "depth name: " << save_depth_name << std::endl;
      std::cout << "rgb  name: " << save_rgb_name << std::endl;

      count++;

      cv::imwrite(save_depth_name, depth);
      cv::imwrite(save_rgb_name, rgb);
    }
  }
}

void test_read_rgb_and_aligndepth()
{
  Camera cam(0);
  cam.Initialize();

  int count = 0;
  cv::Mat align_depth, rgb;
  while (true)
  {
    if (!cam.ReadRGBandAlignDepth(rgb, align_depth))
      continue;

    std::cout << "align_depth resolution: (" << align_depth.rows << ","
              << align_depth.cols << ")\n";
    std::cout << "rgb   resolution: (" << rgb.rows << "," << rgb.cols << ")\n";
    cv::imshow("align_depth", align_depth);
    cv::imshow("rgb", rgb);

    char key = cv::waitKey(50);
    if (key == 'q')
    {
      std::cout << "--------------------------\n";
      break;
    }
    else if (key == 's')
    {
      std::cout << "***********************************\n";
      std::string save_depth_name = "depth_" + std::to_string(count) + ".png";
      std::string save_rgb_name = "rgb_" + std::to_string(count) + ".png";

      std::cout << "depth name: " << save_depth_name << std::endl;
      std::cout << "rgb  name: " << save_rgb_name << std::endl;

      count++;

      cv::imwrite(save_depth_name, align_depth);
      cv::imwrite(save_rgb_name, rgb);
    }
  }
}

void test_export_camera_params()
{
  Camera cam(0);
  cam.Initialize();

  cv::Mat camera_mat, dist_mat;
  camera_mat = cv::Mat::eye(3, 3, CV_32F);
  dist_mat = cv::Mat::zeros(1, 5, CV_32F);

  cam.exportCameraIntrinsicParams(camera_mat, dist_mat);

  std::cout << "camera mat: " << camera_mat << std::endl;
  std::cout << "dist mat: " << dist_mat << std::endl;
}

void test_export_extrinsic_params()
{
  Camera cam(0);
  cam.Initialize();
  cv::Mat extrinsic_mat = cv::Mat::eye(4 , 4 , CV_32F);
  cam.exportCameraExtrinsicParams(extrinsic_mat);

  std::cout << "extrinsic matrix: " << extrinsic_mat << std::endl;

}

TEST(CameraTest, DISABLED_Camera)
{
  // TODO: we need a mock camera for unit test
  Camera cam(0);
  cam.Initialize();
  cv::Mat rgb;
  std::vector<Eigen::Vector3d> pointCloud;
  uint32_t frame = 0;
  while (true)
  {
    auto key = cv::waitKey(10);
    if (key == 27)
    {
      break;
    }
    if (!cam.ReadFrame(rgb, pointCloud))
    {
      continue;
    }
    cv::imwrite("./rgb_" + std::to_string(frame) + ".png", rgb);
    std::ofstream f("./pcd_" + std::to_string(frame) + ".pts");
    for (const auto &p : pointCloud)
    {
      f << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    f.close();
    LOG(INFO) << "Export frame: " << frame << "\n";
    frame++;
  }

  // test_read_rgb();
  // test_read_depth();
  // test_read_rgb_and_depth();

  // test_read_rgb_and_aligndepth();
  // test_export_camera_params();
}
