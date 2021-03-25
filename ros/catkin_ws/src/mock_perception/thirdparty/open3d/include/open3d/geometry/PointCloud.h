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
#include <memory>
#include <tuple>
#include <vector>

#include "open3d/geometry/Image.h"
#include "open3d/geometry/RGBDImage.h"
#include "open3d/camera/PinholeCameraIntrinsic.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/geometry/BoundingVolume.h"

namespace open3d {

namespace geometry {

//class Image;
//class RGBDImage;
/// \class PointCloud
///
/// \brief A point cloud consists of point coordinates, and optionally point
/// colors and point normals.
class PointCloud {
public:
    /// \brief Default Constructor.
    PointCloud() {}
    /// \brief Parameterized Constructor.
    ///
    /// \param points Points coordinates.
    PointCloud(const std::vector<Eigen::Vector3d> &points)
        : points_(points) {}
    ~PointCloud() {}

public:
    PointCloud &Clear();
    bool IsEmpty() const;
    Eigen::Vector3d GetMinBound() const;
    Eigen::Vector3d GetMaxBound() const;
    Eigen::Vector3d GetCenter() const;
    AxisAlignedBoundingBox GetAxisAlignedBoundingBox() const;
    PointCloud &Transform(const Eigen::Matrix4d &transformation);
    PointCloud &Translate(const Eigen::Vector3d &translation,
                          bool relative = true);
    PointCloud &Scale(const double scale,
                      const Eigen::Vector3d &center);
    PointCloud &Rotate(const Eigen::Matrix3d &R,
                       const Eigen::Vector3d &center);

    PointCloud &operator+=(const PointCloud &cloud);
    PointCloud operator+(const PointCloud &cloud) const;

    /// Returns 'true' if the point cloud contains points.
    bool HasPoints() const { return points_.size() > 0; }

    /// Returns `true` if the point cloud contains point normals.
    bool HasNormals() const {
        return points_.size() > 0 && normals_.size() == points_.size();
    }

    /// Returns `true` if the point cloud contains point colors.
    bool HasColors() const {
        return points_.size() > 0 && colors_.size() == points_.size();
    }

    /// Normalize point normals to length 1.
    PointCloud &NormalizeNormals() {
        for (size_t i = 0; i < normals_.size(); i++) {
            normals_[i].normalize();
        }
        return *this;
    }

    /// \brief Remove all points from the point cloud that have a nan entry, or
    /// infinite entries.
    ///
    /// Also removes the corresponding normals and color entries.
    ///
    /// \param remove_nan Remove NaN values from the PointCloud.
    /// \param remove_infinite Remove infinite values from the PointCloud.
    PointCloud &RemoveNonFinitePoints(bool remove_nan = true,
                                      bool remove_infinite = true);

    /// \brief Function to select points from \p input pointcloud into
    /// \p output pointcloud.
    ///
    /// Points with indices in \p indices are selected.
    ///
    /// \param indices Indices of points to be selected.
    /// \param invert Set to `True` to invert the selection of indices.
    std::shared_ptr<PointCloud> SelectByIndex(
            const std::vector<size_t> &indices, bool invert = false) const;


    /// \brief Function to remove points that have less than \p nb_points in a
    /// sphere of a given radius.
    ///
    /// \param nb_points Number of points within the radius.
    /// \param search_radius Radius of the sphere.
    std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
    RemoveRadiusOutliers(size_t nb_points, double search_radius) const;

    /// \brief Function to remove points that are further away from their
    /// \p nb_neighbor neighbors in average.
    ///
    /// \param nb_neighbors Number of neighbors around the target point.
    /// \param std_ratio Standard deviation ratio.
    std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
    RemoveStatisticalOutliers(size_t nb_neighbors, double std_ratio) const;

    /// \brief Factory function to create a pointcloud from a depth image and a
    /// camera model.
    ///
    /// Given depth value d at (u, v) image coordinate, the corresponding 3d
    /// point is: z = d / depth_scale\n x = (u - cx) * z / fx\n y = (v - cy) * z
    /// / fy\n
    ///
    /// \param depth The input depth image can be either a float image, or a
    /// uint16_t image. \param intrinsic Intrinsic parameters of the camera.
    /// \param extrinsic Extrinsic parameters of the camera.
    /// \param depth_scale The depth is scaled by 1 / \p depth_scale.
    /// \param depth_trunc Truncated at \p depth_trunc distance.
    /// \param stride Sampling factor to support coarse point cloud extraction.
    ///
    /// \return An empty pointcloud if the conversion fails.
    /// If \param project_valid_depth_only is true, return point cloud, which
    /// doesn't
    /// have nan point. If the value is false, return point cloud, which has
    /// a point for each pixel, whereas invalid depth results in NaN points.
    static std::shared_ptr<PointCloud> CreateFromDepthImage(
            const Image &depth,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic = Eigen::Matrix4d::Identity(),
            double depth_scale = 1000.0,
            double depth_trunc = 1000.0,
            int stride = 1,
            bool project_valid_depth_only = true);

    /// \brief Factory function to create a pointcloud from an RGB-D image and a
    /// camera model.
    ///
    /// Given depth value d at (u, v) image coordinate, the corresponding 3d
    /// point is: z = d / depth_scale\n x = (u - cx) * z / fx\n y = (v - cy) * z
    /// / fy\n
    ///
    /// \param image The input image.
    /// \param intrinsic Intrinsic parameters of the camera.
    /// \param extrinsic Extrinsic parameters of the camera.
    ///
    /// \return An empty pointcloud if the conversion fails.
    /// If \param project_valid_depth_only is true, return point cloud, which
    /// doesn't
    /// have nan point. If the value is false, return point cloud, which has
    /// a point for each pixel, whereas invalid depth results in NaN points.
    static std::shared_ptr<PointCloud> CreateFromRGBDImage(
            const RGBDImage &image,
            const camera::PinholeCameraIntrinsic &intrinsic,
            const Eigen::Matrix4d &extrinsic = Eigen::Matrix4d::Identity(),
            bool project_valid_depth_only = true);

public:
    /// Points coordinates.
    std::vector<Eigen::Vector3d> points_;
    /// Points normals.
    std::vector<Eigen::Vector3d> normals_;
    /// RGB colors of points.
    std::vector<Eigen::Vector3d> colors_;
};

}  // namespace geometry
}  // namespace open3d
