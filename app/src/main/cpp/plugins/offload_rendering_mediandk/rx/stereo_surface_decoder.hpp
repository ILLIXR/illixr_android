#pragma once

#include "illixr/data_format/frame.hpp"

#include "frame_decoder.hpp"

#include <EGL/egl.h>

namespace ILLIXR {
// Stereo decoder managing left and right eye decoders.
// Provides methods to get decoded frames in dual_frames format.
class stereo_surface_decoder {
public:
    stereo_surface_decoder(int width, int height);

    ~stereo_surface_decoder();

    // Non-copyable
    stereo_surface_decoder(const stereo_surface_decoder&) = delete;

    stereo_surface_decoder& operator=(const stereo_surface_decoder&) = delete;

    // Initialize both decoders.
    bool initialize(JNIEnv* env, EGLDisplay egl_display, EGLContext egl_context);

    // Queue encoded data for an eye.
    // eye: 0=left, 1=right
    bool queue_encoded_data(int eye, const uint8_t* data, size_t size,
                            int64_t timestamp_us, bool is_keyframe);

    // Update textures and get the current frame data.
    // Must be called from GL render thread with context current.
    //
    // This method:
    // 1. Calls updateTexImage() on both SurfaceTextures
    // 2. Packages the texture handles and transforms into a dual_frames struct
    [[nodiscard]] data_format::dual_frames get_current_frame(time_point render_time);

    // Check if both decoders are ready
    [[nodiscard]] bool is_ready() const;

    // Get texture ID for an eye (for direct access if needed)
    [[nodiscard]] GLuint get_texture_id(int eye) const;

    // Flush both decoders
    void flush();

    // Stop both decoders
    void stop();

    // Get frame dimensions
    [[nodiscard]] int get_width() const { return padded_width_; }

    [[maybe_unused]] [[nodiscard]] int get_height() const { return padded_height_; }

private:
    int original_width_;
    int original_height_;
    int padded_width_;
    int padded_height_;
    uint64_t frame_counter_{0};

    std::unique_ptr <frame_decoder> left_decoder_;
    std::unique_ptr <frame_decoder> right_decoder_;
};
}