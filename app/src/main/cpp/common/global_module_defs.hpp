/*
This is the file where default values are defined
*/

#pragma once

#include <string>
#include <stdexcept>


namespace ILLIXR {
	
#ifndef FB_WIDTH
#define FB_WIDTH FB_WIDTH

//Setting default Framebuffer width
static constexpr int FB_WIDTH = 1080; //Pixels
#endif

#ifndef FB_HEIGHT
#define FB_HEIGHT FB_HEIGHT

//Setting default framebuffer height
static constexpr int FB_HEIGHT = 2400; //Pixels
#endif //FB_HEIGHT

/**
 * @brief Convert a string containing a (python) boolean to the bool type
 */
    struct display_params {
            // Display width in pixels
            static constexpr unsigned width_pixels = 2560;

            // Display height in pixels
            static constexpr unsigned height_pixels = 1440;

            // Display width in meters
            static constexpr float width_meters = 0.11047f;

            // Display height in meters
            static constexpr float height_meters = 0.06214f;

            // Separation between lens centers in meters
            static constexpr float lens_separation = width_meters / 2.0f;

            // Vertical position of the lens in meters
            static constexpr float lens_vertical_position = height_meters / 2.0f;

            // Display horizontal field-of-view in degrees
            static constexpr float fov_x = 90.0f;

            // Display vertical field-of-view in degrees
            static constexpr float fov_y = 90.0f;

            // Meters per tangent angle at the center of the HMD (required by timewarp_gl's distortion correction)
            static constexpr float meters_per_tan_angle = width_meters / (2 * (fov_x * M_PI / 180.0f));

            // Inter-pupilary distance (ipd) in meters
            static constexpr float ipd = 0.064f;

            // Display refresh rate in Hz
            static constexpr float frequency = 120.0f;

            // Display period in nanoseconds

            // Chromatic aberration constants
            static constexpr float aberration[4] = {-0.016f, 0.0f, 0.024f, 0.0f};
    };

/// Rendering parameters
    struct rendering_params {
        // Near plane distance in meters
        static constexpr float near_z = 0.1f;

        // Far plane distance in meters
        static constexpr float far_z = 20.0f;
    };


inline bool str_to_bool(std::string var) {
    return (var == "True")  ? true  :
           (var == "False") ? false :
           throw std::runtime_error("Invalid conversion from std::string to bool");
}

/// Temporary environment variable getter. Not needed once #198 is merged.
inline std::string getenv_or(std::string var, std::string default_) {
    if (std::getenv(var.c_str())) {
        return {std::getenv(var.c_str())};
    } else {
        return default_;
    }
}

} /// namespace ILLIXR
