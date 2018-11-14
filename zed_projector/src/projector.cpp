// BSD 3-Clause License
//
// Copyright (c) 2018, UMIGV
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <zed_projector/projector.h>

#include <zed_projector/errors.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <tf2/LinearMath/Vector3.h>

namespace zp {

struct XyzBgr {
    tf2::Vector3 xyz;
    cv::Vec3b bgr;
};

static void open(sl::Camera &camera, sl::InitParameters params);

static std::pair<sl::Mat, PosixClock::time_point> grab_and_retrieve_pointcloud(sl::Camera &camera);

static XyzBgr get_value(const sl::Mat &mat, int x, int y);

static bool can_narrow_to_int(double x) noexcept;

static void grab(sl::Camera &camera);

static PosixClock::time_point get_image_timestamp(sl::Camera &camera);

static sl::Mat retrieve_measure(sl::Camera &camera, sl::MEASURE measure);

Projector::Projector(sl::InitParameters params, ProjectorConfig config)
: resolution_{ config.resolution }, max_height_{ config.max_height },
  rows_{ config.rows }, cols_{ config.cols }
{
    open(camera_, std::move(params));
}

std::pair<cv::Mat, PosixClock::time_point> Projector::project(const tf2::Transform &transform) {
    const auto pc_tp = grab_and_retrieve_pointcloud(camera_);
    const auto &pointcloud = pc_tp.first;

    std::pair<cv::Mat, PosixClock::time_point> to_return{
        std::piecewise_construct,
        std::forward_as_tuple(rows_, cols_, CV_8UC3, cv::Scalar(0, 0, 0)),
        std::forward_as_tuple(pc_tp.second)
    };

    cv::Mat &projected = to_return.first;
    cv::Mat heights{ rows_, cols_, CV_32FC1,
                     cv::Scalar(-std::numeric_limits<float>::infinity()) };

    for (int i = 0; i < static_cast<int>(pointcloud.getHeight()); ++i) {
        for (int j = 0; i < static_cast<int>(pointcloud.getWidth()); ++j) {
            const XyzBgr elem = get_value(pointcloud, i, j);
            const tf2::Vector3 transformed = transform * elem.xyz;

            const auto maybe_ij = get_projected_indices(transformed.x(), transformed.y());

            if (!maybe_ij) {
                continue;
            }

            float &height = heights.at<float>(*maybe_ij);

            if (transformed.z() > max_height_ || transformed.z() <= height) {
                continue;
            }

            height = static_cast<float>(transformed.z());
            projected.at<cv::Vec3b>(*maybe_ij) = elem.bgr;
        }
    }

    return to_return;
}

boost::optional<cv::Vec2i>
Projector::get_projected_indices(double x, double y) const noexcept {
    assert(can_narrow_to_int(x));
    assert(can_narrow_to_int(y));

    const double width = static_cast<double>(cols_) * resolution_;
    const double height = static_cast<double>(rows_) * resolution_;

    const double x_offset = width / 2;
    const double y_offset = height / 2;

    const auto i = static_cast<int>((y + y_offset) / resolution_);
    const auto j = static_cast<int>((x + x_offset) / resolution_);

    if (i < 0 || i >= rows_ || j < 0 || j >= cols_) {
        return boost::none;
    }

    return { { i, j } };
}

void open(sl::Camera &camera, sl::InitParameters params) {
    const std::error_code opened = camera.open(std::move(params));

    if (!opened) {
        throw std::system_error{ opened, "sl::Camera::open" };
    }
}

std::pair<sl::Mat, PosixClock::time_point> grab_and_retrieve_pointcloud(sl::Camera &camera) {
    grab(camera);
    const auto timestamp = get_image_timestamp(camera);
    sl::Mat points = retrieve_measure(camera, sl::MEASURE_XYZRGBA);

    assert(points.getDataType() == sl::MAT_TYPE_32F_C4);

    return { points, timestamp };
}

XyzBgr get_value(const sl::Mat &mat, int i, int j) {
    assert(i >= 0 && i < static_cast<int>(mat.getHeight()));
    assert(j >= 0 && j < static_cast<int>(mat.getWidth()));

    sl::float4 value;
    const std::error_code got = mat.getValue(static_cast<std::size_t>(i),
                                             static_cast<std::size_t>(j), &value);

    if (!got) {
        throw std::system_error{ got, "sl::Mat::getValue" };
    }

    const sl::uchar4 rgba{ reinterpret_cast<const unsigned char*>(&value.w) };

    return { { value.x, value.y, value.z }, { rgba.b, rgba.g, rgba.r } };
}

bool can_narrow_to_int(double x) noexcept {
    static constexpr auto MIN = static_cast<double>(std::numeric_limits<int>::min());
    static constexpr auto MAX = static_cast<double>(std::numeric_limits<int>::max());

    return std::isfinite(x) && x <= MAX && x >= MIN;
}

void grab(sl::Camera &camera) {
    const std::error_code grabbed = camera.grab();

    if (!grabbed) {
        throw std::system_error{ grabbed, "sl::Camera::grab" };
    }
}

PosixClock::time_point get_image_timestamp(sl::Camera &camera) {
    const auto timestamp = camera.getTimestamp(sl::TIME_REFERENCE_IMAGE);

    if (timestamp == 0) {
        throw NoTimestampAvailable{ "zp::get_image_timestamp" };
    }

    return PosixClock::time_point{ PosixClock::duration{ timestamp } };
}

sl::Mat retrieve_measure(sl::Camera &camera, sl::MEASURE measure) {
    sl::Mat data;
    const std::error_code retrieved = camera.retrieveMeasure(data, measure);

    if (!retrieved) {
        throw std::system_error{ retrieved, "sl::Camera::retrieveMeasure" };
    }

    return data;
}

} // namespace zp
