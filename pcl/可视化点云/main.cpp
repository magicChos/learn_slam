#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main( int argc, char * argv[]){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/Users/han/Opencv_Project/pcl/可视化点云/map.pcd" , *cloud) == -1){
        return -1;
    }

    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud , 0 , 0 , 255);
    viewer.addPointCloud(cloud, color_handler , "cloud");
    viewer.spin();

    return 0;
}
