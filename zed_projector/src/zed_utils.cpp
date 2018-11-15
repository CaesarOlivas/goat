#include <zed_projector/zed_utils.h>

#include <zed_projector/errors.h>

namespace zp {

void open(sl::Camera &camera, sl::InitParameters params) {
    const std::error_code opened = camera.open(std::move(params));

    if (opened) {
        throw std::system_error{ opened, "sl::Camera::open" };
    }
}

std::pair<sl::Mat, PosixClock::time_point> grab_and_retrieve_pointcloud(sl::Camera &camera) {
    grab(camera);
    const auto timestamp = get_image_timestamp(camera);
    sl::Mat points = retrieve_measure(camera, sl::MEASURE_XYZRGBA);

    assert(points.getDataType() == sl::MAT_TYPE_32F_C4);

    std::pair<sl::Mat, PosixClock::time_point> to_return{
        std::piecewise_construct,
        std::forward_as_tuple(),
        std::forward_as_tuple(timestamp)
    };

    const std::error_code moved = points.move(to_return.first);

    if (moved) {
        throw std::system_error{ moved, "sl::Mat::move" };
    }

    return to_return;
}

XyzBgr get_value(const sl::Mat &mat, int i, int j) {
    assert(i >= 0 && i < static_cast<int>(mat.getHeight()));
    assert(j >= 0 && j < static_cast<int>(mat.getWidth()));

    sl::float4 value;
    const std::error_code got = mat.getValue(static_cast<std::size_t>(j),
                                             static_cast<std::size_t>(i), &value);

    if (got) {
        throw std::system_error{ got, "sl::Mat::getValue" };
    }

    const sl::uchar4 rgba{ reinterpret_cast<const unsigned char*>(&value.w) };

    return { { value.x, value.y, value.z }, { rgba.b, rgba.g, rgba.r } };
}

void grab(sl::Camera &camera) {
    const std::error_code grabbed = camera.grab();

    if (grabbed) {
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

    if (retrieved) {
        throw std::system_error{ retrieved, "sl::Camera::retrieveMeasure" };
    }

    return data;
}

} // namespace zp
