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
#include <vector>

namespace open3d {
namespace geometry {

/// \class Image
///
/// \brief The Image class stores image with customizable width, height, num of
/// channels and bytes per channel.
class Image{
public:
    /// \enum ColorToIntensityConversionType
    ///
    /// \brief Specifies whether R, G, B channels have the same weight when
    /// converting to intensity. Only used for Image with 3 channels.
    ///
    /// When `Weighted` is used R, G, B channels are weighted according to the
    /// Digital ITU BT.601 standard: I = 0.299 * R + 0.587 * G + 0.114 * B.
    enum class ColorToIntensityConversionType {
        /// R, G, B channels have equal weights.
        Equal,
        /// Weighted R, G, B channels: I = 0.299 * R + 0.587 * G + 0.114 * B.
        Weighted,
    };

    /// \enum FilterType
    ///
    /// \brief Specifies the Image filter type.
    enum class FilterType {
        /// Gaussian filter of size 3 x 3.
        Gaussian3,
        /// Gaussian filter of size 5 x 5.
        Gaussian5,
        /// Gaussian filter of size 7 x 7.
        Gaussian7,
        /// Sobel filter along X-axis.
        Sobel3Dx,
        /// Sobel filter along Y-axis.
        Sobel3Dy
    };

public:
    /// \brief Default Constructor.
    Image(){}
    ~Image(){}

public:
    Image &Clear();
    bool IsEmpty() const;
    Eigen::Vector2d GetMinBound() const;
    Eigen::Vector2d GetMaxBound() const;

public:
    /// Returns `true` if the Image has valid data.
    bool HasData() const {
        return width_ > 0 && height_ > 0 &&
               data_.size() == size_t(height_ * BytesPerLine());
    }

    /// \brief Prepare Image properties and allocate Image buffer.
    Image &Prepare(int width,
                   int height,
                   int num_of_channels,
                   int bytes_per_channel) {
        width_ = width;
        height_ = height;
        num_of_channels_ = num_of_channels;
        bytes_per_channel_ = bytes_per_channel;
        AllocateDataBuffer();
        return *this;
    }

    /// \brief Returns data size per line (row, or the width) in bytes.
    int BytesPerLine() const {
        return width_ * num_of_channels_ * bytes_per_channel_;
    }

    /// Function to access the bilinear interpolated float value of a
    /// (single-channel) float image.
    /// Returns a tuple, where the first bool indicates if the u,v coordinates
    /// are within the image dimensions, and the second double value is the
    /// interpolated pixel value.
    std::pair<bool, double> FloatValueAt(double u, double v) const;
    /// Return a gray scaled float type image.
    std::shared_ptr<Image> CreateFloatImage(
            Image::ColorToIntensityConversionType type =
                    Image::ColorToIntensityConversionType::Weighted) const;

    /// Function to access the raw data of a single-channel Image.
    template <typename T>
    T *PointerAt(int u, int v) const;

    /// Function to access the raw data of a multi-channel Image.
    template <typename T>
    T *PointerAt(int u, int v, int ch) const;

    /// Reinterpret the internal data buffer. The resulting type's size must be
    /// the same as bytes_per_channel_. This is similar to PointerAt<T>(0, 0).
    template <class T>
    T *PointerAs() const {
        if (sizeof(T) != bytes_per_channel_) {
            //utility::LogError("sizeof(T) != byte_per_channel_: {} != {}.", sizeof(T), bytes_per_channel_);
        }
        return (T *)(data_.data());
    }

    std::shared_ptr<Image> ConvertDepthToFloatImage(
            double depth_scale = 1000.0, double depth_trunc = 3.0) const;
    /// Function to change data types of image
    /// crafted for specific usage such as
    /// single channel float image -> 8-bit RGB or 16-bit depth image.
    template <typename T>
    std::shared_ptr<Image> CreateImageFromFloatImage() const;
protected:
    void AllocateDataBuffer() {
        data_.resize(width_ * height_ * num_of_channels_ * bytes_per_channel_);
    }

public:
    /// Width of the image.
    int width_ = 0;
    /// Height of the image.
    int height_ = 0;
    /// Number of chanels in the image.
    int num_of_channels_ = 0;
    /// Number of bytes per channel.
    int bytes_per_channel_ = 0;
    /// Image storage buffer.
    std::vector<uint8_t> data_;
};

}  // namespace geometry
}  // namespace open3d
