#include "utils/debug_utils.h"
#include <set>

void slamMapToMatInv(const nav_messages::FusionOccupancyGrid &map, cv::Mat &map_cv)
{
    int size_x = map.info.width;
    int size_y = map.info.height;

    if ((size_x < 3) || (size_y < 3))
    {
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ((map_cv.rows != size_y) || (map_cv.cols != size_x))
    {
        map_cv = cv::Mat(size_y, size_x, CV_8U);
    }
    const std::vector<int8_t> &map_data(map.data);
    // unsigned char *map_mat_data_p=(unsigned char*) map_cv.data;
    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom

    int size_y_rev = size_y - 1;
    for (int y = size_y_rev; y >= 0; --y)
    {
        int idx_map_y = size_x * (size_y_rev - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x)
        {
            int idx = idx_img_y + x;
            if (map_data[idx_map_y + x] == -1)
            {
                map_cv.data[idx] = 127;
            }
            else if (map_data[idx_map_y + x] < 60)
            {
                map_cv.data[idx] = 0;
            }
            else
            {
                map_cv.data[idx] = map_data[idx_map_y + x];
            }
        }
    }
}


void slamMapToMat(const nav_messages::FusionOccupancyGrid &map, cv::Mat &map_cv)
{
    int size_x = map.info.width;
    int size_y = map.info.height;

    if ((size_x < 3) || (size_y < 3))
    {
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ((map_cv.rows != size_y) || (map_cv.cols != size_x))
    {
        map_cv = cv::Mat(size_y, size_x, CV_8U);
    }
    const std::vector<int8_t> &map_data(map.data);
    // unsigned char *map_mat_data_p=(unsigned char*) map_cv.data;
    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom

    int size_y_rev = size_y - 1;
    for (int y = size_y_rev; y >= 0; --y)
    {
        int idx_map_y = size_x * (size_y_rev - y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x)
        {
            int idx = idx_img_y + x;
            if (map_data[idx_map_y + x] == -1)
            {
                map_cv.data[idx] = 127;
            }
            else if (map_data[idx_map_y + x] >= 60)
            {
                map_cv.data[idx] = 0;
            }
            else
            {
                map_cv.data[idx] = 255;
            }
        }
    }
}

int64_t GetTimeStamp()
{
    auto timeNow = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    return timeNow.count();
}

Eigen::MatrixXd Mat2MatrixXd(const cv::Mat &R)
{
    Eigen::MatrixXd T(R.rows, R.cols);
    cv::cv2eigen(R, T);
    return T;
}

void printTimeStamp(const ImageData &image_data, const CloudData &cloud_data, const geometry_messages::Pose2D &robot_pose)
{
    std::cout << "{-----------------------------------------------------" << std::endl;
    std::cout << "image timestamp: " << image_data.timestamp << std::endl;
    std::cout << "cloud timestamp: " << cloud_data.timestamp << std::endl;
    std::cout << "robot timestamp: " << robot_pose.timestamp << std::endl;
    std::cout << "------------------------------------------------------}" << std::endl;
}

void printRobotPose(const geometry_messages::Pose2D &robot_pose)
{
    std::cout << "robot pose: (" << robot_pose.x << " , " << robot_pose.y << " , " << robot_pose.theta << ")" << std::endl;
}

bool pixelCluster(cv::Mat &img, std::map<uchar, cv::Rect> &cluster_result)
{
    int height = img.rows;
    int width = img.cols;

    cluster_result.clear();

    std::map<uchar, std::vector<cv::Point>> cluster_points;
    for (size_t i = 0; i < height; ++i)
    {
        uchar *p = img.ptr<uchar>(i);
        for (int j = 0; j < width; ++j)
        {
            if (p[j] == 127 || p[j] == 255 || p[j] < 200)
            {
                continue;
            }

            if (p[j] >= 201 && p[j] <= 205)
            {
                cluster_points[p[j]].emplace_back(cv::Point(j, i));
            }
        }
    }

    for (auto iter : cluster_points)
    {
        uchar label = iter.first;
        cv::Rect rect = getRect(iter.second);
        cluster_result[label] = rect;
    }

    return true;
}

cv::Rect getRect(const std::vector<cv::Point> &points)
{
    int xmin = INT_MAX;
    int ymin = INT_MAX;
    int xmax = INT_MIN;
    int ymax = INT_MIN;

    for (auto p : points)
    {
        if (p.x < xmin)
        {
            xmin = p.x;
        }

        if (p.y < ymin)
        {
            ymin = p.y;
        }

        if (p.x > xmax)
        {
            xmax = p.x;
        }

        if (p.y > ymax)
        {
            ymax = p.y;
        }
    }

    return cv::Rect(cv::Point2i(xmin, ymin), cv::Point2i(xmax + 1, ymax + 1));
}