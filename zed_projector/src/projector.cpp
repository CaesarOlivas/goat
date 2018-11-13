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

#include <limits>

namespace zp {

static void open(sl::Camera &camera, sl::InitParameters params);

static std::pair<sl::Mat, PosixClock::time_point> grab_and_retrieve_pointcloud(sl::Camera &camera);

static sl::float4 get_value(const sl::Mat &mat, int x, int y);

static void grab(sl::Camera &camera);

static PosixClock::time_point get_image_timestamp(sl::Camera &camera);

static sl::Mat retrieve_measure(sl::Camera &camera, sl::MEASURE measure);

std::pair<cv::Mat, PosixClock::time_point> Projector::project(const tf2::Transform &transform) {
	const auto pc_tp = grab_and_retrieve_pointcloud(camera_);
	const auto &pointcloud = pc_tp.first;

	std::pair<cv::Mat, PosixClock::time_point> to_return{
		std::piecewise_construct,
		std::forward_as_tuple(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0)),
		std::forward_as_tuple(pc_tp.second)
	};

	cv::Mat &projected = to_return.first;
	cv::Mat heights{ height_, width_, CV_32FC1,
					 cv::Scalar(-std::numeric_limits<float>::infinity()) };

	for (int i = 0; i < static_cast<int>(pointcloud.getHeight()); ++i) {
		for (int j = 0; i < static_cast<int>(pointcloud.getWidth()); ++j) {
			const sl::float4 elem = get_value(pointcloud, i, j);No
		}
	}

	return to_return;
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

static sl::float4 get_value(const sl::Mat &mat, int x, int y) {
	assert(x >= 0 && x < static_cast<int>(mat.getWidth()));
	assert(y >= 0 && y < static_cast<int>(mat.getHeight()));
	sl::float4 value;

	const std::error_code got = mat.getValue(static_cast<std::size_t>(x),
											 static_cast<std::size_t>(y), &value);

	if (!got) {
		throw std::system_error{ got, "sl::Mat::getValue" };
	}

	return value;
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
