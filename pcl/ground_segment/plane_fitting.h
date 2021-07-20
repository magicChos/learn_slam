/************************************************************************************************
@filename    :plane_fitting.cpp
@brief       :ransac平面拟合
@time        :2021/07/04 23:16:25
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>


class FittingPlane
{
public:
    typedef pcl::PointXYZ PointT;
    FittingPlane();
    virtual ~FittingPlane();
    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud);
    Eigen::VectorXf fit(float threshold);

private:
    std::vector<int> m_inliers;
    pcl::SampleConsensusModelPlane<PointT>::Ptr m_model;
};
