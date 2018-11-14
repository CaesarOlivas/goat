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

#ifndef ZED_PROJECTOR_PROJECTOR_H
#define ZED_PROJECTOR_PROJECTOR_H

#include <zed_projector/posix_clock.h>

#include <cstddef>
#include <utility>

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <tf2/LinearMath/Transform.h>

namespace zp {

// defines a 20m x 20m map with 1cm cells
constexpr double DEFAULT_RESOLUTION = 0.01;
constexpr double DEFAULT_MAX_HEIGHT = 1;
constexpr int DEFAULT_ROWS = 2000;
constexpr int DEFAULT_COLS = 2000;

struct ProjectorConfig {
	double resolution = DEFAULT_RESOLUTION;
	double max_height = DEFAULT_MAX_HEIGHT;
	int rows = DEFAULT_ROWS;
	int cols = DEFAULT_COLS;
};

class Projector {
public:
	Projector(sl::InitParameters params, ProjectorConfig config = ProjectorConfig{ });

	std::pair<cv::Mat, PosixClock::time_point> project(const tf2::Transform &transform);

private:
	boost::optional<cv::Vec2i> get_projected_indices(double x, double y) const noexcept;

	sl::Camera camera_;
	double resolution_ = DEFAULT_RESOLUTION; // meters per pixel
	double max_height_ = DEFAULT_MAX_HEIGHT; // max height of points
	int rows_ = DEFAULT_ROWS; // pixels in y direction
	int cols_ = DEFAULT_COLS; // pixels in x direction
};

} // namespace zp

#endif
