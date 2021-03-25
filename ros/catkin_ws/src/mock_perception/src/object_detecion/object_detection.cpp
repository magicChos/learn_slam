#include "object_detection.h"
// #include "gpu.h"
#include "cpu.h"
#include "net.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace ace
{
  namespace perception
  {
    static ncnn::Net *handle = 0;

    int sric_detection_init(const std::string &model_path)
    {
      handle = new ncnn::Net();
      if (handle == 0)
      {
        std::cout << "sric_detection_init failed" << std::endl;
        return -1;
      }

      std::string param_file_name = model_path + "/object.param";
      std::string bin_file_name = model_path + "/object.bin";

      handle->load_param(param_file_name.data());
      handle->load_model(bin_file_name.data());
      return 0;
    }

    static void pre_process(const cv::Mat &img, ncnn::Mat &in, int size)
    {
      in = ncnn::Mat::from_pixels_resize(img.data, ncnn::Mat::PIXEL_BGR2RGB,
                                         img.cols, img.rows, size, size);

      //数据预处理
      const float mean_vals[3] = {0.f, 0.f, 0.f};
      const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
      in.substract_mean_normalize(mean_vals, norm_vals);

      //std::cout << "input normalized" << std::endl;
    }

    void sric_detection_release(void)
    {
      delete handle;
      handle = 0;
    }

    int sric_detection_process(cv::Mat img, std::vector<OBJECT> &objects)
    {
      if (img.empty())
      {
        std::cout << "image is empty ,please check!" << std::endl;
        return -1;
      }
      int size = 320;
      int num_thread = 1;
      float confThresh = 0.9f;
      int img_w = img.cols;
      int img_h = img.rows;

      ncnn::Mat in;
      ncnn::Mat out;
      pre_process(img, in, size);

      ncnn::set_cpu_powersave(2);

      ncnn::Extractor ex = handle->create_extractor();
      ex.set_light_mode(true);
      ex.set_num_threads(num_thread);

      const char *input_blob_name = "data";
      const char *output_blob_name = "output";
      ex.input(input_blob_name, in);
      ex.extract(output_blob_name, out);

      // printf("%d %d %d\n", out.w, out.h, out.c);
      // std::vector<Object> objects;
      objects.clear();
      for (int i = 0; i < out.h; i++)
      {
        const float *values = out.row(i);
        OBJECT object;
        object.label = values[0];
        object.prob = values[1];
        if(object.prob > confThresh)
        {
            object.rect.x = std::max(0, (int)(values[2] * img_w));
            object.rect.y = std::max(0, (int)(values[3] * img_h));
            object.rect.width = std::min((int)(values[4] * img_w), img_w-1) - object.rect.x;
            object.rect.height = std::min((int)(values[5] * img_h), img_h-1)- object.rect.y;
    
            objects.push_back(object);
        }
        
      }
      return objects.size();
    }

  } // namespace perception
} // namespace ace
