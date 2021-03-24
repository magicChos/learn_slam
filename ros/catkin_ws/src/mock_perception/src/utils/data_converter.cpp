#include "utils/data_converter.hpp"

bool convert_point_cloud(const pcl::PointCloud<pcl::PointXYZ> &point_cloud, std::vector<Eigen::Vector3d> &output_point_cloud)
{
    if (point_cloud.empty())
    {
        return false;
    }
    for (auto it = point_cloud.begin(); it != point_cloud.end(); ++it)
    {
        output_point_cloud.emplace_back(Eigen::Vector3d(it->x * 1000, it->y * 1000, it->z * 1000));
    }
    return true;
}