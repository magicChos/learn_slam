#include "utils/debug_utils.h"

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

