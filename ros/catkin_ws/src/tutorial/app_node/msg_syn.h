#ifndef CLIMBING_ROBOT_GT_MAP_
#define CLIMBING_ROBOT_GT_MAP_

#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

namespace climbing_robot
{
    class MsgSynchronizer
    {
    public:
        MsgSynchronizer(ros::NodeHandle node);
        ~MsgSynchronizer(){};

        void callback(const sensor_msgs::PointCloud2::ConstPtr &ori_pointcloud, const sensor_msgs::Image::ConstPtr &ori_image);

        inline void CloseBag()
        {
            msg_syn_bag.close();
        }

    private:
        std::string syn_bag_path;
        rosbag::Bag msg_syn_bag;
    };

} //namespace climbing_robot

#endif // CLIMBING_ROBOT_GT_MAP_