#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <future>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <omp.h>

using namespace std;

const double PI = 3.1415926;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
class Timer
{
	using Clock = std::chrono::high_resolution_clock;

public:
	void start()
	{
		start_ = Clock::now();
	}

	void end()
	{
		end_ = Clock::now();
	}

	double cost_time()
	{
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_);
		return duration.count();
	}

private:
	Clock::time_point start_, end_;
};

template <typename PointT>
void conditionFilter(typename pcl::PointCloud<PointT>::Ptr cloud_dst, const typename pcl::PointCloud<PointT>::Ptr cloud_src, float z = -0.7)
{
	typename pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>()); //创建条件定义对象
	//为条件定义对象添加比较算子: 使用大于0.0和小于0.8这两个条件用于建立滤波器。
	range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GE, -0.55)));

	pcl::ConditionalRemoval<PointT> condrem(range_cond);
	condrem.setInputCloud(cloud_src);
	condrem.setKeepOrganized(true);
	condrem.filter(*cloud_dst);
}

double computePoint2PlaneDist(const Eigen::Vector3d &p3d, const Eigen::Vector4d &plane_coef)
{
	Eigen::Vector3d plane = plane_coef.block<3, 1>(0, 0);

	return (std::fabs(p3d[0] * plane_coef[0] + p3d[1] * plane_coef[1] + p3d[2] * plane_coef[2] + plane_coef[3])) / plane.norm();
}

// 计算拟合平面的类型
// 1: ground 蓝色
// 2: wall	绿色
// 0: unknow
int recongnizeType(const Eigen::Vector3d &plane_norm, const Eigen::Vector3d &ground_norm)
{
	double theta = std::acos(ground_norm.dot(plane_norm) / (ground_norm.norm() * plane_norm.norm())) * 180 / PI;
	std::cout << "theta: " << theta << std::endl;

	if (std::fabs(90 - theta) <= 15)
	{
		std::cout << "该平面是墙面" << std::endl;

		return 2;
	}
	else if (std::abs(theta) <= 15)
	{
		std::cout << "该平面是地面" << std::endl;
		return 1;
	}
	return 0;
}

typedef pcl::PointXYZRGB PointType;
// 计算点云最近点的平均距离
double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x) || !pcl_isfinite((*cloud)[i].y) || !pcl_isfinite((*cloud)[i].z))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

PointCloudPtr filerPassThrough(PointCloudConstPtr cloud_in, const std::string &axis, const float min_val = -0.6, const float max_val = 2.4)
{
    PointCloudPtr res(new PointCloud);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min_val, max_val);
    pass.filter(*res);
    return res;
}

int main(int argc, char **argv)
{
	// 定义地面法向量
	static Eigen::Vector3d ground_norm;
	ground_norm << 0.0, 0.0, 1.0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 记录体素滤波点云数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_removaled(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile("/home/han/Desktop/map_merge.pcd", *cloud);
	size_t point_number = cloud->points.size();
	std::cout << "@test before filter point number: " << point_number << std::endl;

	cloud = filerPassThrough(cloud , "z" , -0.8 , 1.8);

	point_number = cloud->points.size();
	std::cout << "@test after filter point number: " << point_number << std::endl;


	pcl::io::savePCDFile("filter.pcd", *cloud);


// 	std::shared_ptr<Timer> timer_obj = std::make_shared<Timer>();
// 	timer_obj->start();
// 	std::cout << "#### 计算点云密度 ####" << std::endl;
// 	double resolution = computeCloudResolution(cloud);
// 	std::cout << "resolution: " << resolution << std::endl;
// 	timer_obj->end();
// 	double cost_time = timer_obj->cost_time();
// 	std::cout << "计算点云密度 cost time: " << cost_time << std::endl;

// 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
// 	cloud_result->width = point_number;
// 	cloud_result->height = 1;
// 	cloud_result->is_dense = false;
// 	cloud_result->resize(point_number);
// #if defined(_OPENMP)
// #pragma omp parallel for schedule(dynamic)
// #endif
// 	for (int i = 0; i < point_number; ++i)
// 	{
// 		cloud_result->points[i].x = cloud->points[i].x;
// 		cloud_result->points[i].y = cloud->points[i].y;
// 		cloud_result->points[i].z = cloud->points[i].z;

// 		cloud_result->points[i].r = 255;
// 		cloud_result->points[i].g = 255;
// 		cloud_result->points[i].b = 255;
// 	}

// 	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZRGB>);
// 	// conditionFilter<pcl::PointXYZRGB>(cloud_cut, cloud_result);

// 	// pcl::io::savePCDFile("cloud_cut.pcd", *cloud_cut);

// 	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
// 	// // vg.setInputCloud(cloud_cut);
// 	vg.setInputCloud(cloud_result);
// 	vg.setLeafSize(0.01f, 0.01f, 0.01f);
// 	vg.filter(*cloud_filtered);

// 	std::cout << "下采样之后点云数量：" << cloud_filtered->points.size() << std::endl;

// 	std::cout << "#### 下采样之后點雲密度 ####" << std::endl;
// 	resolution = computeCloudResolution(cloud_filtered);
// 	std::cout << "resolution: " << resolution << std::endl;

// 	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
// 	sor.setInputCloud(cloud_filtered);
// 	sor.setMeanK(50);
// 	sor.setStddevMulThresh(1.0);
// 	sor.filter(*cloud_filtered_removaled);

// 	std::cout << "#### 滤波之后剩余点数量： " << cloud_filtered_removaled->points.size() << std::endl;

// 	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// 	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

// 	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
// 	seg.setOptimizeCoefficients(true);
// 	seg.setModelType(pcl::SACMODEL_PLANE);
// 	seg.setMethodType(pcl::SAC_RANSAC);
// 	seg.setDistanceThreshold(0.02);
// 	seg.setMaxIterations(1000);

// 	pcl::PCDWriter writer;
// 	int i = 0;
// 	int nr_points = (int)cloud_filtered_removaled->points.size();

// 	std::vector<Eigen::Vector4d> record_plane_coefs;
// 	while (cloud_filtered_removaled->points.size() > 0.3 * nr_points)
// 	{

// 		seg.setInputCloud(cloud_filtered_removaled);
// 		seg.segment(*inliers, *coefficients);

// 		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
// 		extract.setInputCloud(cloud_filtered_removaled);
// 		extract.setIndices(inliers);
// 		// 如果设为true,可以提取指定index之外的点云
// 		extract.setNegative(false);
// 		extract.filter(*cloud_plane);

// 		if (cloud_plane->points.size() < 10000)
// 			break;

// 		Eigen::Vector4d coef_plane;
// 		coef_plane << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];
// 		record_plane_coefs.emplace_back(coef_plane);

// 		std::stringstream ss;
// 		ss << "full_room_plane_" << i << ".pcd";
// 		writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_plane, false);

// 		double sum_error = 0.0, max_error = 0.0;
// #if defined(_OPENMP)
// #pragma omp parallel for schedule(dynamic)
// #endif
// 		for (int i = 0; i < cloud_plane->points.size(); i++)
// 		{
// 			double tmp_error = std::fabs(coefficients->values[0] * cloud_plane->points[i].x +
// 										 coefficients->values[1] * cloud_plane->points[i].y + coefficients->values[2] * cloud_plane->points[i].z + coefficients->values[3]);
// 			sum_error += tmp_error;
// 			if (max_error < tmp_error)
// 				max_error = tmp_error;
// 		}

// 		cout << "Plane " << i << endl;
// 		cout << "coefficient: " << coefficients->values[0] << " "
// 			 << coefficients->values[1] << " "
// 			 << coefficients->values[2] << " "
// 			 << coefficients->values[3] << endl;
// 		cout << "max_error: " << max_error << endl;
// 		cout << "mean_error: " << sum_error / cloud_plane->points.size() << endl;

// 		extract.setNegative(true);
// 		extract.filter(*cloud_f);
// 		cloud_filtered_removaled.swap(cloud_f);
// 		i++;
// 	}

// 	std::cout << "------------------------------------" << std::endl;
// 	for (auto &coef : record_plane_coefs)
// 	{
// 		int label = recongnizeType(coef.block<3, 1>(0, 0), ground_norm);
// 		std::cout << "label: " << label << std::endl;

// 		for (int i = 0; i < point_number; ++i)
// 		{
// 			Eigen::Vector3d p3d;
// 			p3d << cloud_result->points[i].x, cloud_result->points[i].y, cloud_result->points[i].z;
// 			double dist = computePoint2PlaneDist(p3d, coef);

// 			if (dist < 0.05)
// 			{
// 				if (label == 1)
// 				{
// 					cloud_result->points[i].r = 0;
// 					cloud_result->points[i].g = 0;
// 					cloud_result->points[i].b = 255;
// 				}
// 				else if (label == 2)
// 				{
// 					cloud_result->points[i].r = 0;
// 					cloud_result->points[i].g = 255;
// 					cloud_result->points[i].b = 0;
// 				}
// 			}
// 		}
// 	}

// 	pcl::io::savePCDFile("cloud_result.pcd", *cloud_result);

	return 0;
}
