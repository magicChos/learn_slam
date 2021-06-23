/************************************************************************************************
@filename    :main.cpp
@brief       :possion重建生成mesh
@time        :2021/06/21 12:54:08
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/



#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile("sample.ply", *cloud) == -1)
    {
        PCL_ERROR("Could not read pcd file!\n");
        return 0;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                         //法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    pcl::Poisson<pcl::PointNormal> pn;
    // 设置参数[1,5]，值越大越精细，耗时越久
    pn.setDegree(2);
    pn.setDepth(8);
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    // 是否使用法向量的大小作为置信信息
    pn.setConfidence(false);     //设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
    pn.setManifold(false);       //设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    // pn.setOutputPolygons(true); //设置是否输出为多边形
    // 用户提取ISO等值面的算法的深度
    pn.setIsoDivide(8);
    pn.setSamplesPerNode(3); //设置每个八叉树节点上最少采样点数目
    pn.setSolverDivide(8);

    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);


    // std::cout << "mesh: " << mesh << std::endl;

    // 保存Mesh
    pcl::io::savePLYFile("save_mesh.ply", mesh);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "my");
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}