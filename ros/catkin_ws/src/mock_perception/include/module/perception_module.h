#pragma once
#include "module/base_module.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "lcm_cpp/geometry_messages/Pose2D.hpp"
#include "obstacle_detection/obstacle_detection.h"
#include "sensor_data/obstacle_data.hpp"
#include <vector>
#include "sensor/camera.h"
#include <deque>

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

    void fillLargeMap();

    bool fusion_stragty_resize(nav_messages::FusionOccupancyGrid &fusion_occupancy_grid);

    bool fusion_stragety_recall(const nav_messages::FusionOccupancyGrid &fusion_occupancy_grid, nav_messages::FusionOccupancyGrid &fusion_occupancy_grid_small);
    
    void setTofBaseMatrix(Eigen::Matrix4d &tofBaseMatrix);
private:
    Eigen::Matrix4d m_tof_2_base_matrix;
    Eigen::Matrix4d m_base_2_map_matrix;

    cv::Mat m_global_map, m_local_map;
    geometry_messages::Pose2D m_robot_pose, m_last_robot_pose;
    nav_messages::FusionOccupancyGrid m_occupancy_grid, m_resize_occupancy_grid;
    std::shared_ptr<ace::perception::ObstacleDetector> m_obstacle_detector;
    std::shared_ptr<ace::sensor::CameraInterface> m_camera;

    ace::perception::ObstacleDetectOption m_option;

    // std::deque<ObstaclePoint> m_obstacle_pts;
    std::list<ObstaclePoint> m_obstacle_pts;
    int64_t m_current_timeStamp;

    float m_tof_x = 0.0;
    float m_tof_y = 0.0;


};


