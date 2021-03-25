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

#include <string>
#include <Eigen/Geometry>
#include "open3d/utility/Eigen.h"

namespace open3d {
namespace geometry {


Eigen::Vector3d ComputeMinBound(const std::vector<Eigen::Vector3d>& points);

Eigen::Vector3d ComputeMaxBound(const std::vector<Eigen::Vector3d>& points);

Eigen::Vector3d ComputeCenter(const std::vector<Eigen::Vector3d>& points);

void ResizeAndPaintUniformColor(
        std::vector<Eigen::Vector3d>& colors,
        const size_t size,
        const Eigen::Vector3d& color);

void TransformPoints(const Eigen::Matrix4d& transformation, 
                    std::vector<Eigen::Vector3d>& points);

void TransformNormals(const Eigen::Matrix4d& transformation,
                      std::vector<Eigen::Vector3d>& normals);

void TranslatePoints(const Eigen::Vector3d& translation, 
                     std::vector<Eigen::Vector3d>& points,
                     bool relative);

void ScalePoints(const double scale,
                std::vector<Eigen::Vector3d>& points,
                const Eigen::Vector3d& center);

void RotatePoints(const Eigen::Matrix3d& R,
                  std::vector<Eigen::Vector3d>& points,
                  const Eigen::Vector3d& center);

void RotateNormals(const Eigen::Matrix3d& R, std::vector<Eigen::Vector3d>& normals);


}  // namespace geometry
}  // namespace open3d
