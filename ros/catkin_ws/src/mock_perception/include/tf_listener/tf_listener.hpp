
#pragma once

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "lcm_cpp/geometry_messages/Pose2D.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <deque>

class TFListener
{
public:
    TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    bool LookupData(Eigen::Matrix4d &transform_matrix);

    bool LookupData(geometry_messages::Pose2D &robot_pose);
private:
    bool TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4d &transform_matrix);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};



