#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class YamlReader
{
public:
    YamlReader() = delete;
    YamlReader(const std::string &yaml_file);
    ~YamlReader();

    void getNodeMatrix(const std::string node_name , cv::Mat &mat_val);


private:
    cv::FileStorage storage_;
    
};