// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

namespace open3d {
namespace geometry {

/// Returns the eight points that define the bounding box.
///
///      ------- x
///     /|
///    / |
///   /  | z
///  y
///      0 ------------------- 1
///       /|                /|
///      / |               / |
///     /  |              /  |
///    /   |             /   |
/// 2 ------------------- 7  |
///   |    |____________|____| 6
///   |   /3            |   /
///   |  /              |  /
///   | /               | /
///   |/                |/
/// 5 ------------------- 4
    

/// \class AxisAlignedBoundingBox
///
/// \brief A bounding box that is aligned along the coordinate axes.
///
///  The AxisAlignedBoundingBox uses the cooridnate axes for bounding box
///  generation. This means that the bounding box is oriented along the
///  coordinate axes.
class AxisAlignedBoundingBox
{
public:
    /// \brief Default constructor.
    ///
    /// Creates an empty Axis Aligned Bounding Box.
    AxisAlignedBoundingBox()
        : min_bound_(0, 0, 0),
          max_bound_(0, 0, 0),
          color_(0, 0, 0) {}
    /// \brief Parameterized constructor.
    ///
    /// \param min_bound Lower bounds of the bounding box for all axes.
    /// \param max_bound Upper bounds of the bounding box for all axes.
    AxisAlignedBoundingBox(const Eigen::Vector3d& min_bound,
                           const Eigen::Vector3d& max_bound)
        : min_bound_(min_bound),
          max_bound_(max_bound),
          color_(0, 0, 0) {}
    ~AxisAlignedBoundingBox() {}

public:
    AxisAlignedBoundingBox& Clear();
    bool IsEmpty() const;
    Eigen::Vector3d GetMinBound() const;
    Eigen::Vector3d GetMaxBound() const;
    Eigen::Vector3d GetCenter() const;
    AxisAlignedBoundingBox GetAxisAlignedBoundingBox() const;
    AxisAlignedBoundingBox& Transform(
            const Eigen::Matrix4d& transformation);
    AxisAlignedBoundingBox& Translate(
            const Eigen::Vector3d& translation, bool relative = true);

    /// \brief Scales the axis-aligned bounding boxs.
    /// If \f$mi\f$ is the min_bound and \f$ma\f$ is the max_bound of
    /// the axis aligned bounding box, and \f$s\f$ and \f$c\f$ are the
    /// provided scaling factor and center respectively, then the new
    /// min_bound and max_bound are given by \f$mi = c + s (mi - c)\f$
    /// and \f$ma = c + s (ma - c)\f$.
    ///
    /// \param scale The scale parameter.
    /// \param center Center used for the scaling operation.
    AxisAlignedBoundingBox& Scale(
            const double scale, const Eigen::Vector3d& center);

    /// \brief an AxisAlignedBoundingBox can not be rotated. This method
    /// will throw an error.
    AxisAlignedBoundingBox& Rotate(
            const Eigen::Matrix3d& R, const Eigen::Vector3d& center);

    AxisAlignedBoundingBox& operator+=(const AxisAlignedBoundingBox& other);

    /// Get the extent/length of the bounding box in x, y, and z dimension.
    Eigen::Vector3d GetExtent() const { return (max_bound_ - min_bound_); }

    /// Returns the half extent of the bounding box.
    Eigen::Vector3d GetHalfExtent() const { return GetExtent() * 0.5; }

    /// Returns the maximum extent, i.e. the maximum of X, Y and Z axis'
    /// extents.
    double GetMaxExtent() const { return (max_bound_ - min_bound_).maxCoeff(); }

    double GetXPercentage(double x) const {
        return (x - min_bound_(0)) / (max_bound_(0) - min_bound_(0));
    }

    double GetYPercentage(double y) const {
        return (y - min_bound_(1)) / (max_bound_(1) - min_bound_(1));
    }

    double GetZPercentage(double z) const {
        return (z - min_bound_(2)) / (max_bound_(2) - min_bound_(2));
    }

    /// Returns the volume of the bounding box.
    double Volume() const;
    /// Returns the eight points that define the bounding box.
    std::vector<Eigen::Vector3d> GetBoxPoints() const;

    /// Return indices to points that are within the bounding box.
    ///
    /// \param points A list of points.
    std::vector<size_t> GetPointIndicesWithinBoundingBox(
            const std::vector<Eigen::Vector3d>& points) const;

    /// Returns the 3D dimensions of the bounding box in string format.
    std::string GetPrintInfo() const;

    /// Creates the bounding box that encloses the set of points.
    ///
    /// \param points A list of points.
    static AxisAlignedBoundingBox CreateFromPoints(
            const std::vector<Eigen::Vector3d>& points);

public:
    /// The lower x, y, z bounds of the bounding box.
    Eigen::Vector3d min_bound_;
    /// The upper x, y, z bounds of the bounding box.
    Eigen::Vector3d max_bound_;
    /// The color of the bounding box in RGB.
    Eigen::Vector3d color_;
};

}  // namespace geometry
}  // namespace open3d
