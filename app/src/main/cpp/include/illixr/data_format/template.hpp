#pragma once

#ifndef NDEBUG
#include <spdlog/spdlog.h>
#endif

#include "illixr/data_format/unit.hpp"

namespace ILLIXR::data_format {
/*
 * Normalize the coordinates, using the input size as reference
 */
template<typename T>
inline void normalize(T& obj, const float width, const float height, const float depth) {
    if (obj.unit == units::PERCENT) {
#ifndef NDEBUG
        spdlog::get("illixr")->info("[normalize] already normalized");
#endif
        return;
    }
    obj.x() /= width;
    obj.y() /= height;
    obj.z() /= depth;
    obj.unit = units::PERCENT;
}

template<typename T>
inline void normalize(T& obj, const float width, const float height) {
    normalize<T>(obj, width, height, 1.);
}

/*
 * Denormalize the coordinates, using the input size as reference
 */
template<typename T>
inline void denormalize(T& obj, const float width, const float height, const float depth,
                        units::measurement_unit unit_ = units::PIXEL) {
    if (!obj.valid)
        return;
    if (obj.unit != units::PERCENT) {
#ifndef NDEBUG
        spdlog::get("illixr")->info("[denormalize] already denormalized");
#endif
        return;
    }
    if (unit_ == units::PERCENT || unit_ == units::UNSET)
        throw std::runtime_error("Cannot denormalize to PERCENT");

    obj.x() *= width;
    obj.y() *= height;
    obj.z() *= depth;
    obj.unit = unit_;
}

template<typename T>
inline void denormalize(T& obj, const float width, const float height, units::measurement_unit unit_ = units::PIXEL) {
    denormalize<T>(obj, width, height, 1., unit_);
}
} // namespace ILLIXR::data_format
