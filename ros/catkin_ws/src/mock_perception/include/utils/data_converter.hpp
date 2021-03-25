#pragma once

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lcm_cpp/nav_messages/FusionOccupancyGrid.hpp"
#include <nav_msgs/OccupancyGrid.h>

bool convert_point_cloud(const pcl::PointCloud<pcl::PointXYZ> &point_cloud, std::vector<Eigen::Vector3d> &output_point_cloud);

bool convert_rosOccupancyGraid_FusionOccupancyGrid(const nav_msgs::OccupancyGridConstPtr &msg , nav_messages::FusionOccupancyGrid &fusion_grid);

nav_messages::FusionOccupancyGrid FusionOccupancyGrid_clone(const nav_messages::FusionOccupancyGrid &map);
