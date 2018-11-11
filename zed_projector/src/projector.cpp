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

#include <zed_projector/error_code.h>
#include <zed_projector/mat.h>

namespace zed_projector {

static void open(sl::Camera &camera, sl::InitParameters params) {
	const std::error_code opened = camera.open(std::move(params));

	if (!opened) {
		throw std::system_error{ opened, "sl::Camera::open" };
	}
}

static Mat grab_pointcloud(sl::Camera &camera) {
	const std::error_code grabbed = camera.grab();

	if (!grabbed) {
		throw std::system_error{ grabbed, "sl::Camera::grab" };
	}

	sl::Mat points;
	const std::error_code retrieved = camera.retrieveMeasure(points, sl::MEASURE_XYZRGBA);

	if (!retrieved) {
		throw std::system_error{ retrieved, "sl::Camera::retrieveMeasure" };
	}

	assert(points.getDataType() == sl::MAT_TYPE_32F_C4);

	return Mat{ std::move(points) };
}

} // namespace zed_projector
