#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include "object_detection/defines.h"
namespace ace
{
  namespace perception
  {
    // struct OBJECT
    // {
    //   cv::Rect_<float> rect;
    //   int label;
    //   float prob;
      
    //   int track_id;
    // };
    int sric_detection_init(const std::string &model_path);
    void sric_detection_release(void);
    int sric_detection_process(cv::Mat img, std::vector<OBJECT> &objects);
  } // namespace perception
} // namespace ace
