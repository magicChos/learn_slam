#include "module/perception_module.h"
#include "common/ini_reader.h"
#include "common/log.h"
#include "utils/debug_utils.h"
#include "utils/data_converter.hpp"

PerceptionModule::PerceptionModule()
{
    std::cout << "step into perception module" << std::endl;

    init_params();

    m_camera = std::make_shared<ace::sensor::VirtualCamera>();
    m_obstacle_detector = std::make_shared<ace::perception::ObstacleDetector>(m_camera.get(), m_option);
}

void PerceptionModule::setTofBaseMatrix(Eigen::Matrix4d &tofBaseMatrix)
{
    m_tof_2_base_matrix = tofBaseMatrix;
}

bool PerceptionModule::init_params()
{
    std::string current_dir = stlplus::folder_up(__FILE__);
    std::string config_file = current_dir + "../../config/obstacle_detection.ini";
    std::shared_ptr<ace::common::INIReader> reader = std::make_shared<ace::common::INIReader>(config_file);
    if (reader->ParseError() < 0)
    {
        LogError("Can't load obstacle detection params file");
        exit(0);
    }

    m_option.resolution = reader->GetReal("params", "resolution", 10.0);
    m_option.width = reader->GetInteger("params", "width", 200);
    m_option.height = reader->GetInteger("params", "height", 200);
    m_option.lidarTop = reader->GetReal("params", "lidarTop", 70.0);
    m_option.baseTop = reader->GetReal("params", "baseTop", 30.0);
    m_option.armTop = reader->GetReal("params", "armTop", 0.0);
    m_option.armBottom = reader->GetReal("params", "armBottom", -20.0);
    m_option.baseBottom = reader->GetReal("params", "baseBottom", -30.0);
    m_option.detectObjects = reader->GetBoolean("params", "detectObjects", true);
    m_option.debug = reader->GetBoolean("params", "debug", false);
    m_option.minimumVisibleDistance = reader->GetReal("params", "minimumVisibleDistance", 350.0);
    m_option.maximumVisibleDistance = reader->GetReal("params", "maximumVisibleDistance", 1200.0);
    m_option.hfov = reader->GetReal("params", "hfov", 1.2217304763960306);
    m_option.useTof = reader->GetBoolean("params", "useTof", true);
    m_option.elapse_time = reader->GetInteger("params", "elapse_time", 30000);
    m_option.pix_thresh = reader->GetInteger("params", "pix_thresh", 127);

    {
        m_tof_2_base_matrix << 1.0501503000000001e-02, 2.0494568000000001e-03,
            9.9994278000000003e-01, 2.0858668689999998e-01, -9.9994123000000001e-01,
            2.7064331999999999e-03, 1.0495938999999999e-02, -1.6197355000000000e-02,
            -2.6847673999999999e-03, -9.9999422000000004e-01, 2.0777580999999999e-03,
            5.2435359199999997e-02, 0., 0., 0., 1.;

        m_base_2_map_matrix = Eigen::Matrix4d::Identity();
    }
    m_current_timeStamp = GetTimeStamp();

    return true;
}

void PerceptionModule::fillLargeMap()
{
    // 填充放大后的map
    float rate = static_cast<float>(m_occupancy_grid.info.resolution * 1000 / m_option.resolution);
    float rate_inv = 1 / rate;
    m_resize_occupancy_grid.info.resolution = static_cast<float>(m_option.resolution / 1000);
    m_resize_occupancy_grid.info.height = m_global_map.rows;
    m_resize_occupancy_grid.info.width = m_global_map.cols;
    m_resize_occupancy_grid.data_size = m_global_map.rows * m_global_map.cols;
    m_resize_occupancy_grid.data.resize(m_resize_occupancy_grid.data_size);

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < m_resize_occupancy_grid.data_size; ++i)
    {
        int row = i / m_global_map.cols;
        int col = i % m_global_map.cols;

        int small_row = static_cast<int>(row * rate_inv);
        int small_col = static_cast<int>(col * rate_inv);

        int newIndex = small_row * m_occupancy_grid.info.width + small_col;
        m_resize_occupancy_grid.data[i] = m_occupancy_grid.data[newIndex];
    }

    m_resize_occupancy_grid.info.origin.position.x = m_occupancy_grid.info.origin.position.x;
    m_resize_occupancy_grid.info.origin.position.y = m_occupancy_grid.info.origin.position.y;
    m_resize_occupancy_grid.info.origin.position.z = m_occupancy_grid.info.origin.position.z;
    m_resize_occupancy_grid.info.origin.orientation.w = m_occupancy_grid.info.origin.orientation.w;
    m_resize_occupancy_grid.info.origin.orientation.x = m_occupancy_grid.info.origin.orientation.x;
    m_resize_occupancy_grid.info.origin.orientation.y = m_occupancy_grid.info.origin.orientation.y;
    m_resize_occupancy_grid.info.origin.orientation.z = m_occupancy_grid.info.origin.orientation.z;
}

bool PerceptionModule::run()
{
    std::cout << "process perception module" << std::endl;
    return true;
}

cv::Mat PerceptionModule::run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &slam_map)
{
    float diff_theta = robot_pose.theta - m_last_robot_pose.theta;
    float diff_time = robot_pose.timestamp - m_last_robot_pose.timestamp;

    m_last_robot_pose = robot_pose;
    bool roate_fast_flag = false;
    if (std::fabs(diff_theta / diff_time) * 1000 > 0.1)
    {
        roate_fast_flag = true;
    }

    nav_messages::FusionOccupancyGrid publish_map = FusionOccupancyGrid_clone(slam_map);
    m_tof_x = robot_pose.x;
    m_tof_y = robot_pose.y;

    Eigen::Vector4d tof_in_map_coord =
        m_base_2_map_matrix * m_tof_2_base_matrix * Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d norm_point = tof_in_map_coord.head<3>() / tof_in_map_coord(3);

    m_tof_x = norm_point[0];
    m_tof_y = norm_point[1];

    if (roate_fast_flag)
    {
        std::cout << "@test robot rotate fast" << std::endl;
        return cv::Mat();
    }

    if (!GetLocalMap(rgb_image, pointCloud))
    {
        std::cout << "@test generate localmap failture" << std::endl;
        return cv::Mat();
    }

    m_robot_pose = robot_pose;
    updateGlobalMap(robot_pose, slam_map);

    nav_messages::FusionOccupancyGrid fusion_occupancy_resize_map = FusionOccupancyGrid_clone(m_resize_occupancy_grid);
    fusion_stragty_resize(fusion_occupancy_resize_map);
    fusion_stragety_recall(fusion_occupancy_resize_map, publish_map);

    if (m_option.debug)
    {
        cv::Mat fusion_mat;
        slamMapToMat(fusion_occupancy_resize_map, fusion_mat);
        cv::Mat flip_fusion_mat = fusion_mat.clone();
        cv::flip(flip_fusion_mat, flip_fusion_mat, 0);

        int wx, wy;
        worldToMap(m_robot_pose.x, m_robot_pose.y, wx, wy,
                   fusion_occupancy_resize_map);
        cv::circle(flip_fusion_mat, cv::Point(wx, wy), 3, cv::Scalar(0), 2);

        int wx_n, wy_n;
        float target_x = m_robot_pose.x + 1 * std::cos(m_robot_pose.theta);
        float target_y = m_robot_pose.y + 1 * std::sin(m_robot_pose.theta);
        worldToMap(target_x, target_y, wx_n, wy_n,
                   fusion_occupancy_resize_map);

        cv::line(
            flip_fusion_mat,
            cv::Point(wx, wy),
            cv::Point(wx_n, wy_n),
            cv::Scalar(0), 1);

        int flip_fusion_mat_height, flip_fusion_mat_width;
        flip_fusion_mat_height = flip_fusion_mat.rows;
        flip_fusion_mat_width = flip_fusion_mat.cols;

        while (flip_fusion_mat_height > 1000 || flip_fusion_mat_width > 1000)
        {
            flip_fusion_mat_height = flip_fusion_mat_height >> 1;
            flip_fusion_mat_width = flip_fusion_mat_width >> 1;
        }
        cv::resize(flip_fusion_mat, flip_fusion_mat, cv::Size(flip_fusion_mat_width, flip_fusion_mat_height));
        cv::imshow("flip_fusion_mat", flip_fusion_mat);
    }

    cv::Mat fusion_map;
    // slamMapToMat(publish_map, fusion_map);
    slamMapToMatInv(publish_map, fusion_map);
    cv::flip(fusion_map, fusion_map, 0);
    cv::imshow("fusion_map", fusion_map);
    cv::waitKey(10);

    return fusion_map;
}

bool PerceptionModule::GetLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud)
{
    if (m_obstacle_detector->GenerateLocalMap(rgb_image, pointCloud, m_local_map))
    {

        // std::map<uchar , cv::Rect> cluster_res;
        // pixelCluster(m_local_map , cluster_res);

        // cv::Mat temp = m_local_map.clone();
        // for (auto p: cluster_res)
        // {
        //     auto rect = p.second;
        //     cv::rectangle(temp , rect , cv::Scalar(255) , 1);
        // }
        // cv::imshow("localmap", temp);

        cv::imshow("localmap", m_local_map);
        return true;
    }
    return false;
}

bool PerceptionModule::UpdateMap()
{
    float local_map_resolution = m_option.resolution / 1000.0;
    float local_map_resolution_inv = 1 / local_map_resolution;
    int local_map_width = m_local_map.cols;
    int local_map_height = m_local_map.rows;

    float tof_x = m_tof_x;
    float tof_y = m_tof_y;
    float sin_theta = std::sin(m_robot_pose.theta);
    float cos_theta = std::cos(m_robot_pose.theta);

    cv::flip(m_local_map, m_local_map, 1);
    int64_t time_stamp = GetTimeStamp();

#pragma omp parallel for schedule(dynamic)
    for (int u = 0; u < local_map_height; ++u)
    {
        for (int v = 0; v < local_map_width; ++v)
        {
            float du = (local_map_height - u) * local_map_resolution,
                  dv = (v - local_map_width / 2) * local_map_resolution;
            float gu = tof_x + du * cos_theta - dv * sin_theta;
            float gv = tof_y + du * sin_theta + dv * cos_theta;

            int cgu = std::round((gu - m_occupancy_grid.info.origin.position.x) * local_map_resolution_inv);
            int cgv = std::round((gv - m_occupancy_grid.info.origin.position.y) * local_map_resolution_inv);
            if (cgu < 0 || cgv < 0 || cgu >= m_global_map.cols ||
                cgv >= m_global_map.rows)
            {
                continue;
            }
            auto &local_map_value = m_local_map.at<uchar>(u, v);
            auto &global_map_value = m_global_map.at<uchar>(cgv, cgu);
            if (local_map_value == 127)
            {
                continue;
            }

            if (local_map_value > m_option.pix_thresh)
            {
                // m_obstacle_pts.push_back(ObstaclePoint(gu, gv, local_map_value, time_stamp));
                m_obstacle_pts.emplace_back(ObstaclePoint(gu, gv, local_map_value, time_stamp));
            }

            if (global_map_value == 127)
            {
                global_map_value = local_map_value;
                if (local_map_value > m_option.pix_thresh)
                {
                    if (cgu > 0 && m_global_map.at<uchar>(cgv - 1, cgu) < 200)
                    {
                        m_global_map.at<uchar>(cgv - 1, cgu) = local_map_value;
                    }
                    if (cgv > 0 && m_global_map.at<uchar>(cgv, cgu - 1) < 200)
                    {
                        m_global_map.at<uchar>(cgv, cgu - 1) = local_map_value;
                    }
                    if (cgu < m_global_map.rows &&
                        m_global_map.at<uchar>(cgv + 1, cgu) < 200)
                    {
                        m_global_map.at<uchar>(cgv + 1, cgu) = local_map_value;
                    }

                    if (cgv < m_global_map.cols &&
                        m_global_map.at<uchar>(cgv, cgu + 1) < 200)
                    {
                        m_global_map.at<uchar>(cgv, cgu + 1) = local_map_value;
                    }
                }

                continue;
            }

            if (local_map_value == 0)
            {
                global_map_value = 0;
                if (cgv > 0)
                    m_global_map.at<uchar>(cgv - 1, cgu) = 0;
                if (cgu > 0)
                    m_global_map.at<uchar>(cgv, cgu - 1) = 0;
                if (cgv < m_global_map.rows)
                    m_global_map.at<uchar>(cgv + 1, cgu) = 0;
                if (cgu < m_global_map.cols)
                    m_global_map.at<uchar>(cgv, cgu + 1) = 0;
                continue;
            }
        }
    }

    if (m_option.debug)
    {
        if (m_global_map.rows > 0 && m_global_map.cols > 0)
        {
            cv::Mat global_map_copy;
            m_global_map.copyTo(global_map_copy);
            cv::circle(
                global_map_copy,
                cv::Point((m_robot_pose.x - m_occupancy_grid.info.origin.position.x) /
                              local_map_resolution,
                          (m_robot_pose.y - m_occupancy_grid.info.origin.position.y) /
                              local_map_resolution),
                20, cv::Scalar(255));
            cv::line(
                global_map_copy,
                cv::Point((m_robot_pose.x - m_occupancy_grid.info.origin.position.x) /
                              local_map_resolution,
                          (m_robot_pose.y - m_occupancy_grid.info.origin.position.y) /
                              local_map_resolution),
                cv::Point((m_robot_pose.x + 1 * std::cos(m_robot_pose.theta) -
                           m_occupancy_grid.info.origin.position.x) /
                              local_map_resolution,
                          (m_robot_pose.y + 1 * std::sin(m_robot_pose.theta) -
                           m_occupancy_grid.info.origin.position.y) /
                              local_map_resolution),
                cv::Scalar(255), 1);

            int wx, wy;
            std::cout << "@test before obstacle points number is :" << m_obstacle_pts.size() << std::endl;
            for (auto it = m_obstacle_pts.begin(); it != m_obstacle_pts.end();)
            {
                worldToMap(it->pt_x_, it->pt_y_, wx, wy, m_resize_occupancy_grid);
                // if (global_map_copy.at<uchar>(wy, wx) == 0 && global_map_copy.at<uchar>(wy + 1, wx + 1) == 0 && global_map_copy.at<uchar>(wy - 1, wx - 1) == 0 && global_map_copy.at<uchar>(wy, wx + 1) == 0 && global_map_copy.at<uchar>(wy, wx - 1) == 0 && global_map_copy.at<uchar>(wy + 1, wx) == 0 &&
                //     global_map_copy.at<uchar>(wy - 1, wx) == 0 && global_map_copy.at<uchar>(wy - 1, wx + 1) == 0 && global_map_copy.at<uchar>(wy + 1, wx - 1) == 0)
                if (globalMapUpdateCondition(global_map_copy, wx, wy))
                {
                    m_obstacle_pts.erase(it++);
                }
                else
                {
                    it++;
                    global_map_copy.at<uchar>(wy, wx) = 255;
                }
            }

            std::cout << "@test after obstacle points number is : " << m_obstacle_pts.size() << std::endl;

            int global_map_height = global_map_copy.rows;
            int global_map_width = global_map_copy.cols;
            while (global_map_height >= 1000 || global_map_width >= 1000)
            {
                global_map_height = global_map_height >> 1;
                global_map_width = global_map_width >> 1;
            }

            cv::resize(global_map_copy, global_map_copy, cv::Size(global_map_width, global_map_height));
            cv::imshow("global map", global_map_copy);
            std::cout << "@test show global map " << std::endl;
        }
    }
    return true;
}

bool PerceptionModule::fusion_stragty_resize(nav_messages::FusionOccupancyGrid &fusion_occupancy_grid)
{
    int data_size = fusion_occupancy_grid.data.size();
    int wx, wy;
    m_current_timeStamp = GetTimeStamp();

    for (auto it = m_obstacle_pts.begin(); it != m_obstacle_pts.end();)
    {
        if (m_current_timeStamp - it->time_stamp > m_option.elapse_time)
        {
            worldToMap(it->pt_x_, it->pt_y_, wx, wy, fusion_occupancy_grid);
            int newIndex = wx + fusion_occupancy_grid.info.width * wy;
            if (newIndex >= data_size)
            {
                continue;
            }
            fusion_occupancy_grid.data[newIndex] = 0;

            m_obstacle_pts.erase(it++);
        }
        else
        {
            it++;
        }
    }

    for (auto &p : m_obstacle_pts)
    {
        worldToMap(p.pt_x_, p.pt_y_, wx, wy, fusion_occupancy_grid);
        if (p.pix_val_ > m_option.pix_thresh)
        {
            int newIndex = wx + fusion_occupancy_grid.info.width * wy;
            if (newIndex >= data_size)
            {
                continue;
            }

            // 未知障碍牄1�7
            if (p.pix_val_ == 255)
            {
                fusion_occupancy_grid.data[newIndex] = 100;
            }
            else
            {
                fusion_occupancy_grid.data[newIndex] = p.pix_val_ - 100;
            }
        }
    }

    return true;
}

bool PerceptionModule::fusion_stragety_recall(const nav_messages::FusionOccupancyGrid &fusion_occupancy_grid, nav_messages::FusionOccupancyGrid &fusion_occupancy_grid_small)
{
    int data_size = fusion_occupancy_grid.data_size;
    int slam_resize_map_width = fusion_occupancy_grid.info.width;

    float rate = fusion_occupancy_grid_small.info.resolution / fusion_occupancy_grid.info.resolution;

    std::set<int> m_set;
    for (int i = 0; i < data_size; ++i)
    {
        if (fusion_occupancy_grid.data[i] < 100)
        {
            continue;
        }
        int row = i / slam_resize_map_width;
        int col = i % slam_resize_map_width;

        int small_row = static_cast<int>(row / rate);
        int small_col = static_cast<int>(col / rate);

        int newIndex = small_row * fusion_occupancy_grid_small.info.width + small_col;

        if (fusion_occupancy_grid.data[i] >= 100)
        {
            fusion_occupancy_grid_small.data[newIndex] = fusion_occupancy_grid.data[i];
        }
    }

    return true;
}

bool PerceptionModule::updateGlobalMap(const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &occupacy_grid)
{
    float cos_theta = std::cos(robot_pose.theta);
    float sin_theta = std::sin(robot_pose.theta);

    m_base_2_map_matrix(0, 0) = cos_theta;
    m_base_2_map_matrix(0, 1) = -sin_theta;
    m_base_2_map_matrix(0, 3) = robot_pose.x;
    m_base_2_map_matrix(1, 0) = sin_theta;
    m_base_2_map_matrix(1, 1) = cos_theta;
    m_base_2_map_matrix(1, 3) = robot_pose.y;

    m_occupancy_grid = FusionOccupancyGrid_clone(occupacy_grid);
    float rate = static_cast<float>(occupacy_grid.info.resolution * 1000 / m_option.resolution);
    int global_height = static_cast<int>(occupacy_grid.info.height * rate);
    int global_width = static_cast<int>(occupacy_grid.info.width * rate);

    m_global_map = cv::Mat(global_height, global_width, CV_8UC1, cv::Scalar(127));

    fillLargeMap();
    int wx, wy;
    for (auto p : m_obstacle_pts)
    {
        worldToMap(p.pt_x_, p.pt_y_, wx, wy, m_resize_occupancy_grid);
        m_global_map.at<uchar>(wy, wx) = p.pix_val_;
    }

    UpdateMap();

    std::cout << "@test updateGlobalmap here! " << std::endl;
    return true;
}

bool PerceptionModule::fusion_process(cv::Mat &fusion_map)
{
    return true;
}

bool PerceptionModule::globalMapUpdateCondition(const cv::Mat &global_map_img, const int wx, const int wy)
{
    if (global_map_img.at<uchar>(wy, wx) == 0 && global_map_img.at<uchar>(wy + 1, wx) == 0 && global_map_img.at<uchar>(wy + 2, wx) == 0 &&
        global_map_img.at<uchar>(wy - 1, wx) == 0 && global_map_img.at<uchar>(wy - 2, wx) == 0)
    {
        return true;
    }

    return false;
}