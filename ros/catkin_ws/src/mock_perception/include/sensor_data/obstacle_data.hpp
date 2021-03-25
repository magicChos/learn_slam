#pragma once
#include <iostream>

class ObstaclePoint
{
public:
    ObstaclePoint() = default;
    ObstaclePoint(const float pt_x, const float pt_y, const int pix_val, const int64_t time_stamp_)
        : pt_x_(pt_x), pt_y_(pt_y), pix_val_(pix_val), time_stamp(time_stamp_)
    {
    }

    friend std::ostream &operator<<(std::ostream &os, const ObstaclePoint &op)
    {
        os << "x: " << op.pt_x_ << " , y: " << op.pt_y_ << " , pix_val: " << op.pix_val_ << " , timeStamp: " << op.time_stamp << std::endl;
        return os;
    }

public:
    float pt_x_;
    float pt_y_;
    int pix_val_;
    int64_t time_stamp;
};