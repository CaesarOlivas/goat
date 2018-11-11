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

#include <zed_projector/mat.h>

static int get_cv_type(const sl::Mat &mat);

namespace zed_projector {

Mat::Mat(sl::Mat &&data) noexcept
: cv::Mat{ static_cast<int>(data.getHeight()), static_cast<int>(data.getWidth()),
		   get_cv_type(data), data.getPtr<sl::uchar1>(data.getMemoryType()) }
{
	data.move(owner_);
}

} // namespace zed_projector

int get_cv_type(const sl::Mat &mat) {
	switch (mat.getDataType()) {
		case sl::MAT_TYPE_32F_C1: return CV_32FC1;
        case sl::MAT_TYPE_32F_C2: return CV_32FC2;
        case sl::MAT_TYPE_32F_C3: return CV_32FC3;
        case sl::MAT_TYPE_32F_C4: return CV_32FC4;
        case sl::MAT_TYPE_8U_C1: return CV_8UC1;
        case sl::MAT_TYPE_8U_C2: return CV_8UC2;
        case sl::MAT_TYPE_8U_C3: return CV_8UC3;
        case sl::MAT_TYPE_8U_C4: return CV_8UC4;
    }
}
