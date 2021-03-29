#include "system.h"
#include "utils/data_converter.hpp"
#include <stdio.h>
#include "utils/debug_utils.h"

MockSystem::MockSystem(std::shared_ptr<BaseModule> module)
{
    std::cout << "step into mock system" << std::endl;
    m_module = module;

    m_color_sub = std::make_shared<ColorSubscriber>(m_nh, "/pico_camera/color_image", 10000);
    m_cloud_sub = std::make_shared<CloudSubscriber>(m_nh, "/pico_camera/point_cloud", 10000);
    m_map_sub = std::make_shared<MapSubscriber>(m_nh, "/map", 10000);
    m_listener = std::make_shared<TFListener>(m_nh, "map", "base_footprint");

    m_robot_thread = std::make_shared<std::thread>(&MockSystem::handleRobotPose, this);
    m_process_thread = std::make_shared<std::thread>(&MockSystem::Run, this);
}

MockSystem::~MockSystem()
{
}

void MockSystem::Run()
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        usleep(1000);
        ros::spinOnce();
        if (!m_map_sub->parse_data(m_occupancy_grid))
        {
            // std::cout << "@test receive slam map failture" << std::endl;
            continue;
        }
        read_data();

        while (has_data())
        {
            if (!check_data())
            {
                continue;
            }

            // printTimeStamp(m_current_image_data , m_current_cloud_data , m_robot_pose);
            // cv::Mat slam_mat;
            // slamMapToMat(m_occupancy_grid , slam_mat);
            // cv::imshow("slam" , slam_mat);
            // cv::waitKey(100);
            printRobotPose(m_robot_pose);
            cv::Mat fusion_map = m_module->run(m_current_image_data.image, m_current_cloud_vector_data, m_robot_pose, m_occupancy_grid);
        }

        rate.sleep();
    }
}

void MockSystem::handleRobotPose()
{
    ros::Rate rate(300);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_messages::Pose2D robot_pose;
        if (m_listener->LookupData(robot_pose))
        {
            std::lock_guard<std::mutex> robot_pose_mutex(m_robot_pose_mutex);
            m_robot_pose_buffer.emplace_back(robot_pose);
            // std::cout << "robot timestamp: " << robot_pose.timestamp << std::endl;
        }

        rate.sleep();
    }
}

void MockSystem::handleMapMessage(const nav_msgs::OccupancyGridConstPtr &msg)
{
    if (msg == nullptr)
    {
        return;
    }

    if (m_occupancy_grid.data.size() == msg->data.size() || msg->data.size() == 0)
    {
        return;
    }

    std::lock_guard<std::mutex> guard(m_occupancy_grid_mutex);
    convert_rosOccupancyGraid_FusionOccupancyGrid(msg, m_occupancy_grid);

    receive_slammap = true;
    return;
}

bool MockSystem::read_data()
{
    m_color_sub->parse_data(m_image_buffer);
    m_cloud_sub->parse_data(m_cloud_buffer);
    return true;
}

bool MockSystem::has_data()
{
    if (m_image_buffer.empty())
    {
        return false;
    }

    if (m_cloud_buffer.empty())
    {
        return false;
    }

    if (m_robot_pose_buffer.empty())
    {
        return false;
    }

    return true;
}

bool MockSystem::check_data()
{
    m_current_image_data = m_image_buffer.front();
    m_current_cloud_data = m_cloud_buffer.front();
    m_robot_pose = m_robot_pose_buffer.front();

    int diff_cloud_time = m_current_cloud_data.timestamp - m_current_image_data.timestamp;
    int diff_robot_time = m_current_cloud_data.timestamp - m_robot_pose.timestamp;
    if (diff_cloud_time < -50)
    {
        m_cloud_buffer.pop_front();
        return false;
    }

    if (diff_cloud_time > 50)
    {

        m_image_buffer.pop_front();
        return false;
    }

    if (abs(diff_robot_time) > 30)
    {
        m_robot_pose_buffer.pop_front();
        return false;
    }

    if (abs(diff_robot_time) > 10)
    {
        sign_theta_rapid_change_ = 1;
    }
    else
    {
        sign_theta_rapid_change_ = 0;
    }

    convert_point_cloud(*m_current_cloud_data.cloud_ptr, m_current_cloud_vector_data);
    m_image_buffer.pop_front();
    m_cloud_buffer.pop_front();
    m_robot_pose_buffer.pop_front();
    return true;
}