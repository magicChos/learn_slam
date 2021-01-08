/************************************************************************************************
@filename    :show_image_node.cpp
@brief       :接受图像topic并显示
@time        :2021/01/06 00:21:47
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <iostream>

class ImageSubscriber
{
public:
    ImageSubscriber() = delete;
    ImageSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buffer_size) : nh_(nh)
    {
        image_transport::ImageTransport it(nh_);
        sub_ = it.subscribe(topic_name, buffer_size, &ImageSubscriber::msg_callback, this);
    }
    ~ImageSubscriber() {}

public:
    void parse_data(std::deque<cv::Mat> &image_data_buff)
    {
        if (m_img_data_.size() > 0)
        {
            image_data_buff.insert(image_data_buff.end(), m_img_data_.begin(), m_img_data_.end());
        }
    }

private:
    void msg_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        m_img_data_.push_back(img);

        cv::imshow("img" , img);
        cv::waitKey(30);
    }

private:
    std::deque<cv::Mat> m_img_data_;
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_;
};

int main(int argc, char **argv)
{
    std::cout << "-----------------------------------\n";
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    std::shared_ptr<ImageSubscriber> image_subscriber = std::make_shared<ImageSubscriber>(nh, "/pico_camera/color_image", 10);
    std::deque<cv::Mat> img_buffer;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        image_subscriber->parse_data(img_buffer);
        while(img_buffer.size() > 0)
        {
            cv::Mat pop_img = img_buffer.front();
            img_buffer.pop_front();
        }
        

        rate.sleep();
    }
    // ros::spin();
    return 0;
}
