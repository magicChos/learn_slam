#include "plane_fitting.h"

FittingPlane::FittingPlane()
{
}

FittingPlane::~FittingPlane()
{
}

void FittingPlane::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    m_model = boost::make_shared<pcl::SampleConsensusModelPlane<PointT>>(cloud);
}

Eigen::VectorXf FittingPlane::fit(float threshold)
{
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(m_model);
    ransac.setDistanceThreshold(threshold);
    ransac.computeModel();
    ransac.getInliers(m_inliers);

    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    return coeff;
}