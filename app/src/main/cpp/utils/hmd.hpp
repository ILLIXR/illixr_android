#pragma once

#include <array>
#include <EGL/egl.h>
#include <vector>
#include <GLES/gl.h>

// HMD utility class for warp mesh structs, spline math, etc
class HMD {
public:
    static constexpr int NUM_EYES = 2;
    static constexpr int NUM_COLOR_CHANNELS = 3;

    struct mesh_coord2d_t {
        GLfloat x;
        GLfloat y;
    };

    struct mesh_coord3d_t {
        GLfloat x;
        GLfloat y;
        GLfloat z;
    };

    struct uv_coord_t {
        GLfloat u;
        GLfloat v;
    };

    struct hmd_info_t {
        int displayPixelsWide;
        int displayPixelsHigh;
        int tilePixelsWide;
        int tilePixelsHigh;
        int eyeTilesWide;
        int eyeTilesHigh;
        int visiblePixelsWide;
        int visiblePixelsHigh;
        float visibleMetersWide;
        float visibleMetersHigh;
        float lensSeparationInMeters;
        float metersPerTanAngleAtCenter;
        int numKnots;
        float K[11];
        float chromaticAberration[4];
    };

    static float max_float(float x, float y);

    static float min_float(float x, float y);

    static float evaluate_catmull_rom_spline(float value, const float* K, int num_knots);

    [[maybe_unused]] static void
    get_default_hmd_info(const int display_pixels_wide, const int display_pixels_high,
                         const float display_meters_wide, const float display_meters_high,
                         const float lens_separation, const float meters_per_tan_angle,
                         const float aberration[4], hmd_info_t& hmd_info);

    [[maybe_unused]] static void
    build_distortion_meshes(
            std::array<std::array<std::vector<mesh_coord2d_t>, NUM_COLOR_CHANNELS>, NUM_EYES>& distort_coords,
            hmd_info_t& hmd_info);
};
