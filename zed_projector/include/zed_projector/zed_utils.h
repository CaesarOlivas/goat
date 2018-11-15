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

#ifndef ZED_PROJECTOR_ZED_UTILS_H
#define ZED_PROJECTOR_ZED_UTILS_H

#include <zed_projector/posix_clock.h>

#include <utility>

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <tf2/LinearMath/Vector3.h>

namespace zp {

struct XyzBgr {
    tf2::Vector3 xyz;
    cv::Vec3b bgr;
};

void open(sl::Camera &camera, sl::InitParameters params);

std::pair<sl::Mat, PosixClock::time_point> grab_and_retrieve_pointcloud(sl::Camera &camera);

XyzBgr get_value(const sl::Mat &mat, int x, int y);

void grab(sl::Camera &camera);

PosixClock::time_point get_image_timestamp(sl::Camera &camera);

sl::Mat retrieve_measure(sl::Camera &camera, sl::MEASURE measure);

} // namespace zp

#endif
