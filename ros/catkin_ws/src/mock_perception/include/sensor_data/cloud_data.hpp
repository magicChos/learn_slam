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

public:
    int64_t timestamp = 0;
    CLOUD_PTR cloud_ptr;
};