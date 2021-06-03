#pragma once

#include "module/base_module.h"
#include "subscriber/color_subscriber.h"
#include "subscriber/cloud_subscriber.h"
#include "subscriber/map_subscriber.h"
#include "tf_listener/tf_listener.hpp"
#include <memory>
#include <thread>
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/image_data.hpp"
#include <deque>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>
#include "lcm_cpp/nav_messages/FusionOccupancyGrid.hpp"
#include "utils/perception_time.h"
#include "common/log.h"

using namespace ace::common;

class MockSystem
{
public:
    MockSystem(std::shared_ptr<BaseModule> module);
    ~MockSystem();
    void Run();

    void handleRobotPose();

    void handleMapMessage(const nav_msgs::OccupancyGridConstPtr &msg);

    static bool robotPoseSyncData(std::deque<geometry_messages::Pose2D> &unsyncdata , const int64_t sync_time , std::deque<geometry_messages::Pose2D> &syncdata);

protected:
    std::shared_ptr<BaseModule> m_module;

private:
    bool read_data();

    bool has_data();

    bool check_data();

private:
    Eigen::Matrix4d m_tof_base_matrix;
    std::shared_ptr<ColorSubscriber> m_color_sub;
    std::shared_ptr<CloudSubscriber> m_cloud_sub;
    std::shared_ptr<MapSubscriber> m_map_sub;
    // ros::Subscriber m_map_sub;
    std::shared_ptr<TFListener> m_listener , m_tof_listener;

    ros::NodeHandle m_nh;
    std::shared_ptr<std::thread> m_process_thread;
    std::shared_ptr<std::thread> m_robot_thread;
    geometry_messages::Pose2D m_robot_pose;

    std::deque<ImageData> m_image_buffer;
    std::deque<CloudData> m_cloud_buffer;
    std::deque<geometry_messages::Pose2D> m_robot_pose_buffer;

    std::mutex m_robot_pose_mutex;

    ImageData m_current_image_data;
    CloudData m_current_cloud_data;
    std::vector<Eigen::Vector3d> m_current_cloud_vector_data;
    geometry_messages::Pose2D m_current_robot_pose;

    // nav_msgs::OccupancyGrid m_slamMap;
    nav_messages::FusionOccupancyGrid m_occupancy_grid;
    nav_messages::FusionOccupancyGrid m_resize_occupancy_grid;
    std::mutex m_occupancy_grid_mutex;

    bool receive_slammap = false;
    int sign_theta_rapid_change_ = -1;

    std::shared_ptr<Timer> m_timer;

};