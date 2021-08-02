#include "msg_syn.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msg_synchronizer");
    ros::NodeHandle node("~");
    ROS_INFO("\033[1;32m---->\033[0m Sync msgs node Started.");

    climbing_robot::MsgSynchronizer synchronizer(node);

    // ros::spin();

    synchronizer.CloseBag();

    return 0;
}