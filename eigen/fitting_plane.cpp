/************************************************************************************************
@filename    :fitting_plane.cpp
@brief       :最小二乘平面拟合
@time        :2021/02/02 23:11:13
@author      :hscoder
@versions    :1.0
@email       :hscoder@163.com
@usage       :
***********************************************************************************************/

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <random>
#include <numeric>

// Find the plane such that the summed squared distance from the
// plane to all points is minimized.
//
// Reference:
// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
Eigen::Vector4d GetPlaneFromPoints(const std::vector<Eigen::Vector3d> &points,
                                   const std::vector<size_t> &inliers)
{
    Eigen::Vector3d centroid(0, 0, 0);
    for (size_t idx : inliers)
    {
        centroid += points[idx];
    }
    centroid /= double(inliers.size());

    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

    for (size_t idx : inliers)
    {
        Eigen::Vector3d r = points[idx] - centroid;
        xx += r(0) * r(0);
        xy += r(0) * r(1);
        xz += r(0) * r(2);
        yy += r(1) * r(1);
        yz += r(1) * r(2);
        zz += r(2) * r(2);
    }

    double det_x = yy * zz - yz * yz;
    double det_y = xx * zz - xz * xz;
    double det_z = xx * yy - xy * xy;

    Eigen::Vector3d abc;
    if (det_x > det_y && det_x > det_z)
    {
        abc = Eigen::Vector3d(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
    }
    else if (det_y > det_z)
    {
        abc = Eigen::Vector3d(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
    }
    else
    {
        abc = Eigen::Vector3d(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
    }

    double norm = abc.norm();
    // Return invalid plane if the points don't span a plane.
    if (norm == 0)
    {
        return Eigen::Vector4d(0, 0, 0, 0);
    }
    abc /= abc.norm();
    double d = -abc.dot(centroid);
    return Eigen::Vector4d(abc(0), abc(1), abc(2), d);
}

class RANSACResult
{
public:
    RANSACResult() : fitness_(0), inlier_rmse_(0) {}
    ~RANSACResult() {}

public:
    double fitness_;
    double inlier_rmse_;
};

// Calculates the number of inliers given a list of points and a plane model,
// and the total distance between the inliers and the plane. These numbers are
// then used to evaluate how well the plane model fits the given points.
RANSACResult EvaluateRANSACBasedOnDistance(
    const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector4d plane_model,
    std::vector<size_t> &inliers,
    double distance_threshold,
    double error)
{
    RANSACResult result;

    for (size_t idx = 0; idx < points.size(); ++idx)
    {
        Eigen::Vector4d point(points[idx](0), points[idx](1), points[idx](2),
                              1);
        double distance = std::abs(plane_model.dot(point));

        if (distance < distance_threshold)
        {
            error += distance;
            inliers.emplace_back(idx);
        }
    }

    size_t inlier_num = inliers.size();
    if (inlier_num == 0)
    {
        result.fitness_ = 0;
        result.inlier_rmse_ = 0;
    }
    else
    {
        result.fitness_ = (double)inlier_num / (double)points.size();
        result.inlier_rmse_ = error / std::sqrt((double)inlier_num);
    }
    return result;
}

// 过三点计算一个平面方程
Eigen::Vector4d ComputeTrianglePlane(const Eigen::Vector3d &p0,
                                     const Eigen::Vector3d &p1,
                                     const Eigen::Vector3d &p2)
{
    const Eigen::Vector3d e0 = p1 - p0;
    const Eigen::Vector3d e1 = p2 - p0;
    Eigen::Vector3d abc = e0.cross(e1);
    double norm = abc.norm();
    // if the three points are co-linear, return invalid plane
    if (norm == 0)
    {
        return Eigen::Vector4d(0, 0, 0, 0);
    }
    abc /= abc.norm();
    double d = -abc.dot(p0);
    return Eigen::Vector4d(abc(0), abc(1), abc(2), d);
}

void mockData(std::vector<Eigen::Vector3d> &point_cloud, std::vector<size_t> &inliers)
{
    std::random_device rd;
    std::mt19937 rng(rd());
    for (int i = 0; i < 100; ++i)
    {
        Eigen::Vector3d pt;
        pt << rng() % 10, rng() % 10, 6;
        point_cloud.emplace_back(pt);
        inliers.push_back(i);
    }
}

int main(int argc, char **argv)
{
    std::vector<Eigen::Vector3d> pointcloud;
    std::vector<size_t> inliders;
    mockData(pointcloud, inliders);

    Eigen::Vector4d coef = GetPlaneFromPoints(pointcloud, inliders);
    std::cout << "平面方程： " << coef << std::endl;
    return 1;
}