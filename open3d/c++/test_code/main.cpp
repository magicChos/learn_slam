#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <open3d/Open3D.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

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

bool radisuFilter(const std::vector<Eigen::Vector3d> &input_pointCloud, std::vector<Eigen::Vector3d> &output_pointCloud, int size = 10, float radius = 0.01)
{

    // std::vector<Eigen::Vector3d> temp_pointCloud;
    // for (auto &p : input_pointCloud)
    // {
    //     temp_pointCloud.emplace_back(Eigen::Vector3d(p.x(), p.y(), p.z()));
    // }

    open3d::geometry::PointCloud pointcloud_obj;
    pointcloud_obj.points_ = input_pointCloud;
    auto filter = pointcloud_obj.RemoveRadiusOutliers(size, radius);

    size_t reserve_number = std::get<0>(filter)->points_.size();
// #pragma omp parallel for schedule(static)
//     for (auto &p : std::get<0>(filter)->points_)
//     {
//         output_pointCloud.emplace_back(Eigen::Vector3f(p.x(), p.y(), p.z()));
//     }

    output_pointCloud = std::get<0>(filter)->points_;

    return true;
}

int main(int argc, char *argv[])
{
    std::cout << "Hello, Open3D!! " << std::endl;

    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);

    auto pcd = open3d::io::CreatePointCloudFromFile(argv[1]);
    std::cout << pcd->points_.size() << std::endl;

    std::vector<Eigen::Vector3f> pointcloud_f;
    int number = pcd->points_.size();
    for (int i = 0; i < number; ++i)
    {
        auto &pt = pcd->points_[i];
        pointcloud_f.emplace_back(Eigen::Vector3f(pt.x(), pt.y(), pt.z()));
    }

    std::cout << "before filter: " << pointcloud_f.size() << std::endl;

    std::shared_ptr<Timer> timer_obj = std::make_shared<Timer>();
    timer_obj->start();

    std::vector<Eigen::Vector3d> filter_pointCloud;
    radisuFilter(pcd->points_, filter_pointCloud , 10);
    std::cout << "filter after: " << filter_pointCloud.size() << std::endl;

    timer_obj->end();
    double cost_time = timer_obj->cost_time();
    std::cout << "cost time: " << cost_time << std::endl;
    // 1. test downsample
    // auto downsampled = pcd->VoxelDownSample(0.05);
    // {
    //     open3d::utility::ScopeTimer timer("FPFH estimation with Radius 0.25");
    //     open3d::registration::ComputeFPFHFeature(*downsampled, open3d::geometry::KDTreeSearchParamRadius(0.25));
    // }
    // // 2. 估计点云的法向量
    // {
    //     open3d::utility::ScopeTimer timer("Normal estimation with KNN20");
    //     for (int i = 0; i < 20; i++)
    //     {
    //         downsampled->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));
    //     }
    // }
    // std::cout << downsampled->normals_[0] << std::endl;
    // std::cout << downsampled->normals_[10] << std::endl;
    // {
    //     open3d::utility::ScopeTimer timer("Normal estimation with Radius 0.01666");
    //     for (int i = 0; i < 20; i++)
    //     {
    //         downsampled->EstimateNormals(open3d::geometry::KDTreeSearchParamRadius(0.01666));
    //     }
    // }
    // std::cout << downsampled->normals_[0] << std::endl;
    // std::cout << downsampled->normals_[10] << std::endl;

    // open3d::visualization::DrawGeometries({downsampled}, "TestPCD", 1920, 1080);

    return 0;
}
