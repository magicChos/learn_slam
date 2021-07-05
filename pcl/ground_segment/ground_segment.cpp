/************************************************************************************************
@filename    :ground_segment.cpp
@brief       :近似形态学滤波器地面提取
@time        :2021/07/04 22:32:31
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <memory>
#include "plane_fitting.h"

class GroundSegment
{
public:
    typedef pcl::PointXYZ PointT;
    GroundSegment(int maxWindowsize = 20, float slopeThreshold = 1.0f, float initHeight = 0.2f, float maxDistance = 0.05f) : m_inputCloud(new pcl::PointCloud<PointT>),
                                                                                                                             m_maxWindowSize(maxWindowsize),
                                                                                                                             m_slopeThreshold(slopeThreshold),
                                                                                                                             m_initHeight(initHeight),
                                                                                                                             m_maxDistance(maxDistance),
                                                                                                                             m_ground(new pcl::PointIndices)
    {
    }
    virtual ~GroundSegment()
    {
        std::vector<int>().swap(m_ground_indices);
    }
    void setInputCloud(pcl::PointCloud<PointT>::Ptr input_cloud)
    {
        m_inputCloud = input_cloud;
        m_pmf.setInputCloud(m_inputCloud);
    }

    pcl::PointIndicesPtr segment()
    {
        m_ground_indices.clear();
        m_ground.reset();
        m_ground = boost::make_shared<pcl::PointIndices>();

        // 设置过滤点最大的窗口尺寸
        m_pmf.setMaxWindowSize(m_maxWindowSize);

        // 设置计算高度阈值的斜率值
        m_pmf.setSlope(m_slopeThreshold);

        // 设置初始高度参数被认为是地面点
        m_pmf.setInitialDistance(m_initHeight);

        // 设置被认为是地面点的最大高度
        m_pmf.setMaxDistance(m_maxDistance);
        m_pmf.extract(m_ground->indices);
        m_ground_indices = m_ground->indices;

        return m_ground;
    }

    void getGroundIndices(std::vector<int> &indices) const
    {
        indices = m_ground_indices;
    }

    void extractCloud(pcl::PointCloud<PointT>::Ptr extract_cloud)
    {
        m_extract.setInputCloud(m_inputCloud);
        m_extract.setIndices(m_ground);
        m_extract.filter(*extract_cloud);
    }

    // flag: true  提取非地面点
    // flag: false 提取地面点
    void setNegative(bool flag = false)
    {
        m_extract.setNegative(flag);
    }

private:
    pcl::PointCloud<PointT>::Ptr m_inputCloud;
    pcl::ApproximateProgressiveMorphologicalFilter<PointT> m_pmf;
    pcl::PointIndicesPtr m_ground;
    pcl::ExtractIndices<PointT> m_extract;
    int m_maxWindowSize;
    float m_slopeThreshold;
    float m_initHeight;
    float m_maxDistance;
    std::vector<int> m_ground_indices;
};

double computePoint2PlaneDist(const Eigen::Vector3d &p3d, const Eigen::Vector4d &plane_coef)
{
    Eigen::Vector3d plane = plane_coef.block<3, 1>(0, 0);

    return (std::fabs(p3d[0] * plane_coef[0] + p3d[1] * plane_coef[1] + p3d[2] * plane_coef[2] + plane_coef[3])) / plane.norm();
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("1.pcd", *cloud);

    std::shared_ptr<GroundSegment> ground_segment = std::make_shared<GroundSegment>();
    ground_segment->setInputCloud(cloud);
    ground_segment->segment();
    ground_segment->extractCloud(cloud_filtered);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    std::cout << " --------------------------------------------------" << std::endl;
    std::cout << "plane coeff" << std::endl;
    std::shared_ptr<FittingPlane> fitting_plane = std::make_shared<FittingPlane>();
    fitting_plane->setInputCloud(cloud_filtered);
    Eigen::VectorXf coeff = fitting_plane->fit(0.05);

    std::cout << "coeff " << coeff[0] << " " << coeff[1] << " " << coeff[2] << " " << coeff[3] << std::endl;

    Eigen::Vector4d coeff_d = coeff.cast<double>();
    std::cout << "coeff_d: " << coeff_d << std::endl;

    size_t point_number = cloud->points.size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_render(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_render->resize(point_number);

    for (size_t i = 0; i < point_number; ++i)
    {
        Eigen::Vector3d p3d;
        p3d << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
        double dist = computePoint2PlaneDist(p3d, coeff_d);

        cloud_render->points[i].x = cloud->points[i].x;
        cloud_render->points[i].y = cloud->points[i].y;
        cloud_render->points[i].z = cloud->points[i].z;
        cloud_render->points[i].r = 255;
        cloud_render->points[i].g = 255;
        cloud_render->points[i].b = 255;
        if (dist < 0.05)
        {
            cloud_render->points[i].r = 0;
            cloud_render->points[i].g = 255;
            cloud_render->points[i].b = 0;
        }
    }

    pcl::io::savePCDFileBinary("result.pcd", *cloud_render);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    // cloud_filtered_RGB->resize(cloud_filtered->size());
    // cloud_filtered_RGB->is_dense = false;

    // for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    // {
    //     cloud_filtered_RGB->points[i].x = cloud_filtered->points[i].x;
    //     cloud_filtered_RGB->points[i].y = cloud_filtered->points[i].y;
    //     cloud_filtered_RGB->points[i].z = cloud_filtered->points[i].z;

    //     cloud_filtered_RGB->points[i].r = 0;
    //     cloud_filtered_RGB->points[i].g = 255;
    //     cloud_filtered_RGB->points[i].b = 0;
    // }

    // pcl::io::savePCDFileBinary("cloud_groud.pcd", *cloud_filtered_RGB);
    // ground_segment->setNegative(true);
    // ground_segment->extractCloud(cloud_filtered);

    // pcl::io::savePCDFileBinary("No_groud.pcd", *cloud_filtered);

    return (0);
}