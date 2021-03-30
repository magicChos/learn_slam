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

bool convert_rosOccupancyGraid_FusionOccupancyGrid(const nav_msgs::OccupancyGridConstPtr &msg, nav_messages::FusionOccupancyGrid &fusion_grid)
{
    fusion_grid.info.resolution = msg->info.resolution;
    fusion_grid.info.height = msg->info.height;
    fusion_grid.info.width = msg->info.width;

    fusion_grid.data_size = msg->data.size();
    fusion_grid.data.resize(fusion_grid.data_size);
    fusion_grid.data = msg->data;

    fusion_grid.floor_type = '0';
    fusion_grid.obj_size = 0;

    fusion_grid.info.origin.position.x = msg->info.origin.position.x;
    fusion_grid.info.origin.position.y = msg->info.origin.position.y;
    fusion_grid.info.origin.position.z = msg->info.origin.position.z;
    fusion_grid.info.origin.orientation.w = msg->info.origin.orientation.w;
    fusion_grid.info.origin.orientation.x = msg->info.origin.orientation.x;
    fusion_grid.info.origin.orientation.y = msg->info.origin.orientation.y;
    fusion_grid.info.origin.orientation.z = msg->info.origin.orientation.z;

    return true;
}

nav_messages::FusionOccupancyGrid FusionOccupancyGrid_clone(const nav_messages::FusionOccupancyGrid &map)
{
    nav_messages::FusionOccupancyGrid temp;
    temp.info.resolution = map.info.resolution;
    temp.info.height = map.info.height;
    temp.info.width = map.info.width;
    temp.data_size = map.data.size();
    temp.data.resize(temp.data_size);
    temp.data = map.data;
    temp.floor_type = map.floor_type;
    temp.obj_size = map.obj_size;

    temp.info.origin.position.x = map.info.origin.position.x;
    temp.info.origin.position.y = map.info.origin.position.y;
    temp.info.origin.position.z = map.info.origin.position.z;
    temp.info.origin.orientation.w = map.info.origin.orientation.w;
    temp.info.origin.orientation.x = map.info.origin.orientation.x;
    temp.info.origin.orientation.y = map.info.origin.orientation.y;
    temp.info.origin.orientation.z = map.info.origin.orientation.z;

    return temp;
}

bool worldToMap(double wx, double wy, int &mx, int &my, nav_messages::FusionOccupancyGrid &map)
{
    if (wx < map.info.origin.position.x || wy < map.info.origin.position.y)
        return false;

    mx = (int)((wx - map.info.origin.position.x) / map.info.resolution);
    my = (int)((wy - map.info.origin.position.y) / map.info.resolution);

    if (mx < map.info.width && my < map.info.height)
        return true;

    return false;
}