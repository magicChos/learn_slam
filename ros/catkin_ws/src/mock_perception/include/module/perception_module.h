#include "module/base_module.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "lcm_cpp/geometry_messages/Pose2D.hpp"
#include "obstacle_detection/obstacle_detection.h"

class ObstacleDetector;
class ObstacleDetectOption;

class PerceptionModule : public BaseModule
{
public:
    PerceptionModule();
    bool run();

    cv::Mat run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose , const nav_messages::FusionOccupancyGrid &slam_map) ;

    
private:
    bool GetLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud , cv::Mat &local_map);
    bool updateGlobalMap(const geometry_messages::Pose2D &robot_pose);

    bool fusion_process(cv::Mat &fusion_map);
private:
    cv::Mat m_global_map;

    std::shared_ptr<ace::perception::ObstacleDetector> m_obstacle_detector;
};









