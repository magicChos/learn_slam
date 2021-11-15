/************************************************************************************************
@filename    :experiment_for_perception.cpp
@brief       :为了perception的程序
@time        :2021/11/15 13:21:30
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

bool readPointCloudFromFile(pcl::PointCloud<PointT>::Ptr cloud, const std::string &filename)
{
    if (pcl::io::loadPCDFile(filename, *cloud) < 0)
    {
        std::cout << "Error load pointcloud" << std::endl;
        return false;
    }
    return true;
}

bool savePointCloudToFile(const pcl::PointCloud<PointT>::Ptr cloud, const std::string &filename)
{
    if (pcl::io::savePCDFile(filename, *cloud) < 0)
    {
        std::cout << "save pointcloud failed" << std::endl;
        return false;
    }
    return true;
}

PointCloudPtr filterPointCloud(const pcl::PointCloud<PointT>::Ptr src_cloud, const pcl::PointCloud<PointT>::Ptr render_cloud, const int type_id)
{
    PointCloudPtr res(new PointCloud);
    for (int i = 0, number = src_cloud->points.size(); i < number; ++i)
    {
        int r = render_cloud->points[i].r;
        int g = render_cloud->points[i].g;
        int b = render_cloud->points[i].b;

        // blue
        if (type_id == 1)
        {
            if (r == 0 && g == 0 && b == 255)
            {
                res->points.emplace_back(render_cloud->points[i]);
                continue;
            }
        }
        else if (type_id == 2) // red
        {
            if (r == 255 && g == 0 && b == 0)
            {
                res->points.emplace_back(render_cloud->points[i]);
                continue;
            }
            else if (r == 0 && g == 0 && b == 255)
            {
                continue;
            }
        }
        else if (type_id == 3) // green
        {
            if (r == 0 && g == 255 && b == 0)
            {
                res->points.emplace_back(render_cloud->points[i]);
                continue;
            }
            else if (r == 0 && g == 0 && b == 255)
            {
                continue;
            }
        }

        res->points.emplace_back(src_cloud->points[i]);
    }

    res->width = res->points.size();
    res->height = 1;
    res->is_dense = false;

    return res;
}

int main(int argc, char *argv[])
{
    const std::string src_filename = "/home/han/Desktop/map_merge.pcd";
    const std::string render_filename = "/home/han/Desktop/cloud_result.pcd";

    PointCloudPtr src_cloud(new PointCloud);
    PointCloudPtr render_cloud(new PointCloud);

    readPointCloudFromFile(src_cloud, src_filename);
    readPointCloudFromFile(render_cloud, render_filename);

    std::cout << "src pointcloud size = " << src_cloud->points.size() << std::endl;
    std::cout << "render point cloud size = " << render_cloud->points.size() << std::endl;

    // remove roof

    auto roof_cloud = filterPointCloud(src_cloud, render_cloud, 1);
    savePointCloudToFile(roof_cloud, "roof_cloud.pcd");

    // remove ground
    auto ground_cloud = filterPointCloud(src_cloud, render_cloud,3);
    savePointCloudToFile(ground_cloud , "ground_cloud.pcd");

    // remove wall
    auto wall_cloud = filterPointCloud(src_cloud, render_cloud,2);
    savePointCloudToFile(wall_cloud , "wall_cloud.pcd");

    return 1;
}