#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool convert_point_cloud(const pcl::PointCloud<pcl::PointXYZ> &point_cloud, std::vector<Eigen::Vector3d> &output_point_cloud);
