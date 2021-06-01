
#include "tf_listener/tf_listener.hpp"
#include <Eigen/Geometry>

TFListener::TFListener(ros::NodeHandle &nh, std::string base_frame_id, std::string child_frame_id)
    : nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id)
{
}

bool TFListener::LookupData(Eigen::Matrix4d &transform_matrix)
{
    while (true)
    {
        try
        {
            tf::StampedTransform transform;
            listener_.waitForTransform(base_frame_id_, child_frame_id_, ros::Time(0), ros::Duration(3.0));
            listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
            TransformToMatrix(transform, transform_matrix);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    return true;
}

bool TFListener::LookupData(geometry_messages::Pose2D &robot_pose)
{
    try
    {
        tf::StampedTransform transform;
        // listener_.waitForTransform(base_frame_id_, child_frame_id_, ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);

        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
        robot_pose.x = transform.getOrigin().getX();
        robot_pose.y = transform.getOrigin().getY();
        robot_pose.theta = yaw;
        robot_pose.timestamp = transform.stamp_.toNSec() * 0.000001;
        return true;
    }
    catch (tf::TransformException &ex)
    {
        return false;
    }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4d &transform_matrix)
{
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix().cast<double>();

    return true;
}