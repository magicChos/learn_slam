#include "module/base_module.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "lcm_cpp/geometry_messages/Pose2D.hpp"
#include "obstacle_detection/obstacle_detection.h"
#include "sensor_data/obstacle_data.hpp"
#include <vector>

class ObstacleDetector;
class ObstacleDetectOption;

class PerceptionModule : public BaseModule
{
public:
    PerceptionModule();
    bool run();

    cv::Mat run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &slam_map);

private:
    bool GetLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud);
    bool updateGlobalMap(const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &occupacy_grid);

    bool UpdateMap();

    bool fusion_process(cv::Mat &fusion_map);

    bool init_params();

private:
    Eigen::Matrix4d m_tof_2_base_matrix;
    Eigen::Matrix4d m_base_2_map_matrix;

    cv::Mat m_global_map, m_local_map;
    geometry_messages::Pose2D m_robot_pose;
    nav_messages::FusionOccupancyGrid m_occupancy_grid;
    std::shared_ptr<ace::perception::ObstacleDetector> m_obstacle_detector;
    ace::perception::ObstacleDetectOption m_option;

    std::vector<ObstaclePoint> m_obstacle_pts;
};
