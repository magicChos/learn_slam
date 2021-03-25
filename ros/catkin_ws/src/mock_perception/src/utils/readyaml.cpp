#include "readyaml.h"

YamlReader::YamlReader(const std::string &yaml_file)
{
    storage_ = cv::FileStorage(yaml_file, cv::FileStorage::READ);
}

YamlReader::~YamlReader()
{
    storage_.release();
}

void YamlReader::getNodeMatrix(const std::string node_name, cv::Mat &mat_val)
{
    mat_val = storage_[node_name].mat();
}
