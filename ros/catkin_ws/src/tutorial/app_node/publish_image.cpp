#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);


    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        return -1;
    }

    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);

    sensor_msgs::Image img_msg;
    img_msg.header.frame_id = "camera/color_image_raw";
    img_msg.width = width;
    img_msg.height = height;
    img_msg.is_bigendian = false;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_msg.step = img_msg.width * 3;
    int len = img_msg.width * img_msg.height * 3;
    img_msg.data.resize(len);

    std_msgs::Header header;
    header.frame_id = "camera/color_image_raw";
    header.stamp = ros::Time::now();


    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    
    cv::Mat frame;
    ros::Rate loop_rate(25);
    while (nh.ok())
    {
        cap >> frame;
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}