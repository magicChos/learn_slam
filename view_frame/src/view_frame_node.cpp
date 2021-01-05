#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
using namespace Eigen;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tfshow");
    cout << "start program" << endl;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(1, 1, 1));
    tf::Quaternion q;
    q.setRPY(M_PI / 2, M_PI, M_PI / 2);
    transform.setRotation(q);

    int k = 0;
    while (k < 100)
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));
        k++;
        sleep(1);
    }
}
