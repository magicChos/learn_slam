#include "utils/debug_utils.h"
#include <set>

std::vector<cv::Scalar> rgb_colors = {cv::Scalar(246, 42, 42), cv::Scalar(254, 218, 2), cv::Scalar(85, 254, 18), cv::Scalar(10, 250, 147), cv::Scalar(17, 150, 250), cv::Scalar(91, 30, 245), cv::Scalar(244, 44, 216)};

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

void slamMapToMatColor(const nav_messages::FusionOccupancyGrid &map, cv::Mat &map_cv)
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
        map_cv = cv::Mat(size_y, size_x, CV_8UC3);
    }
    const std::vector<int8_t> &map_data(map.data);

    int size_y_rev = size_y - 1;
    for (int y = size_y_rev; y >= 0; --y)
    {
        int idx_map_y = size_x * (size_y_rev - y);
        // int idx_img_y = size_x * y;

        cv::Vec3b *p = map_cv.ptr<cv::Vec3b>(y);

        for (int x = 0; x < size_x; ++x)
        {
            cv::Vec3b &pix = *p++;

            // int idx = idx_img_y + x;
            if (map_data[idx_map_y + x] == -1)
            {
                pix[0] = 127;
                pix[1] = 127;
                pix[2] = 127;
            }
            else if (map_data[idx_map_y + x] >= 60)
            {
                if (map_data[idx_map_y + x] >= 101)
                {
                    int label = map_data[idx_map_y + x] - 101;
                    cv::Scalar color = rgb_colors[label];
                    pix[0] = color[0];
                    pix[1] = color[1];
                    pix[2] = color[2];
                }
                else
                {
                    pix[0] = 0;
                    pix[1] = 0;
                    pix[2] = 0;
                }
            }
            else
            {
                pix[0] = 255;
                pix[1] = 255;
                pix[2] = 255;
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
    for (int i = 0; i < height; ++i)
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

double blurValue(cv::Mat &img)
{
    cv::Mat imageGrey;
    cv::cvtColor(img, imageGrey, cv::COLOR_RGB2GRAY);
    cv::Mat imageSobel;
    cv::Laplacian(imageGrey, imageSobel, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(imageSobel, mean, stddev);
    double meanValue = std::pow(stddev.val[0], 2);

    return meanValue;
}

int project_to_rgb_image(const std::vector<OBJECT> &objects,
                         const Eigen::Vector3f &p3d,
                         const Eigen::Matrix3f &intrinsic_matrix,
                         const Eigen::Matrix3f extrinsic_matrix,
                         const Eigen::Vector3f translation)
{
    Eigen::Vector3f p3c = extrinsic_matrix * p3d + translation;
    p3c /= p3c[2];
    Eigen::Vector3f pix = intrinsic_matrix * p3c;

    for(const auto obj : objects)
    {
        if (pix[0] < obj.rect.x || pix[0] >= obj.rect.x + obj.rect.width || pix[1] < obj.rect.y || pix[1] >= obj.rect.y + obj.rect.height)
        {
            continue;
        }

        return obj.label;
    }

    return -1;
}