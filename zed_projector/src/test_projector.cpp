#include <zed_projector/projector.h>

#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <sl/Camera.hpp>

int main() {
	const tf2::Transform eye{ tf2::Quaternion::getIdentity() };

	sl::InitParameters params;
	params.camera_resolution = sl::RESOLUTION_VGA;
	params.camera_fps = 30;
	params.depth_mode = sl::DEPTH_MODE_ULTRA;
	params.coordinate_units = sl::UNIT_METER;
	params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD;

	zp::Projector projector{ std::move(params) };
	const auto img_stamp = projector.project(eye);
}
