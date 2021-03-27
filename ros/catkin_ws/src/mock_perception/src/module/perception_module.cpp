#include "module/perception_module.h"
#include "common/ini_reader.h"
#include "common/log.h"
#include "utils/debug_utils.h"

PerceptionModule::PerceptionModule()
{
    std::cout << "step into perception module" << std::endl;

    init_params();

    m_camera = std::make_shared<ace::sensor::VirtualCamera>();
    m_obstacle_detector = std::make_shared<ace::perception::ObstacleDetector>(m_camera.get(), m_option);
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
    m_option.visualize = reader->GetBoolean("params", "visualize", false);
    m_option.debug = reader->GetBoolean("params", "debug", false);
    m_option.minimumVisibleDistance = reader->GetReal("params", "minimumVisibleDistance", 350.0);
    m_option.maximumVisibleDistance = reader->GetReal("params", "maximumVisibleDistance", 1200.0);
    m_option.hfov = reader->GetReal("params", "hfov", 1.2217304763960306);
    m_option.useTof = reader->GetBoolean("params", "useTof", true);
    m_option.elapse_time = reader->GetInteger("params", "elapse_time", 30000);

    {
        m_tof_2_base_matrix << 1.0501503000000001e-02, 2.0494568000000001e-03,
            9.9994278000000003e-01, 2.0858668689999998e-01, -9.9994123000000001e-01,
            2.7064331999999999e-03, 1.0495938999999999e-02, -1.6197355000000000e-02,
            -2.6847673999999999e-03, -9.9999422000000004e-01, 2.0777580999999999e-03,
            5.2435359199999997e-02, 0., 0., 0., 1.;

        m_base_2_map_matrix = Eigen::Matrix4d::Identity();
    }

    return true;
}

bool PerceptionModule::run()
{
    std::cout << "process perception module" << std::endl;
    return true;
}

cv::Mat PerceptionModule::run(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud, const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &slam_map)
{
    if (!GetLocalMap(rgb_image, pointCloud))
    {
        std::cout << "@test generate localmap failture" << std::endl;
        return cv::Mat();
    }
    m_robot_pose = robot_pose;
    updateGlobalMap(robot_pose, slam_map);

    // cv::Mat fusion_map;
    // fusion_process(fusion_map);

    // std::cout << "finished" << std::endl;

    // return fusion_map;

    return cv::Mat();
}

bool PerceptionModule::GetLocalMap(const cv::Mat &rgb_image, const std::vector<Eigen::Vector3d> &pointCloud)
{
    m_obstacle_detector->GenerateLocalMap(rgb_image, pointCloud, m_local_map);

    cv::imshow("localmap" , m_local_map);
    cv::waitKey(100);
    return true;
}

bool PerceptionModule::UpdateMap()
{
    float local_map_resolution = m_option.resolution / 1000.0;
    int local_map_width = m_local_map.cols;
    int local_map_height = m_local_map.rows;

    float tof_x = m_robot_pose.x, tof_y = m_robot_pose.y;

    Eigen::Vector4d tof_in_map_coord =
        m_base_2_map_matrix * m_tof_2_base_matrix * Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d norm_point = tof_in_map_coord.head<3>() / tof_in_map_coord(3);

    tof_x = norm_point[0];
    tof_y = norm_point[1];

    cv::flip(m_local_map, m_local_map, 1);

    for (int u = 0; u < local_map_height; ++u)
    {
        for (int v = 0; v < local_map_width; ++v)
        {
            float du = (local_map_height - u) * local_map_resolution,
                  dv = (v - local_map_width / 2) * local_map_resolution;
            float gu = tof_x + du * std::cos(m_robot_pose.theta) -
                       dv * std::sin(m_robot_pose.theta);
            float gv = tof_y + du * std::sin(m_robot_pose.theta) +
                       dv * std::cos(m_robot_pose.theta);

            int cgu = std::round((gu - m_occupancy_grid.info.origin.position.x) /
                                 local_map_resolution);
            int cgv = std::round((gv - m_occupancy_grid.info.origin.position.y) /
                                 local_map_resolution);
            if (cgu < 0 || cgv < 0 || cgu >= m_global_map.cols ||
                cgv >= m_global_map.rows)
            {
                continue;
            }
            auto &local_map_value = m_local_map.at<uchar>(u, v);
            if (local_map_value == 127)
            {
                continue;
            }

            if (local_map_value >= 200)
            {
                int64_t time_stamp = GetTimeStamp();
                m_obstacle_pts.push_back(ObstaclePoint(gu, gv, local_map_value, time_stamp));
            }
        }
    }
    return true;
}

bool PerceptionModule::updateGlobalMap(const geometry_messages::Pose2D &robot_pose, const nav_messages::FusionOccupancyGrid &occupacy_grid)
{
    if (m_occupancy_grid.data.size() == occupacy_grid.data.size() || occupacy_grid.data.size() == 0)
    {
    }
    else
    {
        float rate = static_cast<float>(occupacy_grid.info.resolution * 1000 / m_option.resolution);
        int global_height = static_cast<int>(occupacy_grid.info.height * rate);
        int global_width = static_cast<int>(occupacy_grid.info.width * rate);
        m_global_map = cv::Mat(global_height, global_width, CV_8UC1, cv::Scalar(127));
    }

    UpdateMap();

    return true;
}

bool PerceptionModule::fusion_process(cv::Mat &fusion_map)
{
    return true;
}
