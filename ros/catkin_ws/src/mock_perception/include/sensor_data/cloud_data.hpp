/************************************************************************************************
@filename    :cloud_data.hpp
@brief       :
@time        :2021/03/25 10:04:01
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/
#pragma once
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>

class CloudData
{
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

public:
    CloudData()
        : cloud_ptr(new CLOUD())
    {
    }

    static bool SyncData(std::deque<CloudData> &unsyncData , const double sync_time , std::deque<CloudData> &syncData)
    {
        while(unsyncData.size() >= 2)
        {
            // 如果队列头数据时间比同步时间晚
            if (unsyncData.front().timestamp - sync_time)
            {
                return false;
            }
            // 如果队列第二个数据比同步时间晚，则弹出第一个数据
            if (unsyncData.at(1).timestamp < sync_time)
            {
                unsyncData.pop_front();
                continue;
            }

            if (sync_time - unsyncData.front().timestamp > 100)
            {
                unsyncData.pop_front();
                break;
            }

            if (unsyncData.at(1).timestamp - sync_time > 100)
            {
                unsyncData.pop_front();
                break;
            }
            break;
        }

        if (unsyncData.size() < 2)
        {
            return false;
        }
        CloudData sync_data = unsyncData.front();
        syncData.push_back(sync_data);
    }

public:
    int64_t timestamp = 0;
    CLOUD_PTR cloud_ptr;
};
