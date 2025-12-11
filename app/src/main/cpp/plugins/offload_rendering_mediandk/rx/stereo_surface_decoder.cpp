#include "stereo_surface_decoder.hpp"

using namespace ILLIXR;
using namespace ILLIXR::data_format;

stereo_surface_decoder::stereo_surface_decoder(int width, int height)
        : original_width_{width}
        , original_height_{height}
        , padded_width_{(width + 31) & ~31}
        , padded_height_{(height + 31) & ~31} { // h265 requires 32 bit alignment
    left_decoder_ = std::make_unique<frame_decoder>(0, padded_width_, padded_height_);
    right_decoder_ = std::make_unique<frame_decoder>(1, padded_width_, padded_height_);
}

stereo_surface_decoder::~stereo_surface_decoder() {
    stop();
}

bool stereo_surface_decoder::initialize(JNIEnv* env, EGLDisplay egl_display, EGLContext egl_context) {
    if (!left_decoder_->initialize(env, egl_display, egl_context)) {
        spdlog::get("illixr")->error("stereo_surface_decoder: Left decoder init failed");
        return false;
    }

    if (!right_decoder_->initialize(env, egl_display, egl_context)) {
        spdlog::get("illixr")->error("stereo_surface_decoder: Right decoder init failed");
        left_decoder_->stop();
        return false;
    }

    spdlog::get("illixr")->info("stereo_surface_decoder: Initialized {}x{} for image {}x{}", padded_width_, padded_height_,
                                original_width_, original_height_);
    return true;
}

bool stereo_surface_decoder::queue_encoded_data(int eye, const uint8_t* data, size_t size,
                                                int64_t timestamp_us, bool is_keyframe) {
    if (eye == 0) {
        return left_decoder_->queue_encoded_data(data, size, timestamp_us, is_keyframe);
    } else if (eye == 1) {
        return right_decoder_->queue_encoded_data(data, size, timestamp_us, is_keyframe);
    }
    return false;
}

dual_frames stereo_surface_decoder::get_current_frame(time_point render_time) {
    // Update both textures
    left_decoder_->update_texture();
    right_decoder_->update_texture();

    // Get transform matrices
    float left_transform[16];
    float right_transform[16];
    left_decoder_->get_transform_matrix(left_transform);
    right_decoder_->get_transform_matrix(right_transform);

    // Create dual_frames with texture handles
    dual_frames frames(
            render_time,
            left_decoder_->get_texture_id(), left_transform,
            right_decoder_->get_texture_id(), right_transform,
            padded_width_, padded_height_, frame_counter_++
    );

    return frames;
}

bool stereo_surface_decoder::is_ready() const {
    return left_decoder_->is_ready() && right_decoder_->is_ready();
}

GLuint stereo_surface_decoder::get_texture_id(int eye) const {
    if (eye == 0) return left_decoder_->get_texture_id();
    if (eye == 1) return right_decoder_->get_texture_id();
    return 0;
}

void stereo_surface_decoder::flush() {
    left_decoder_->flush();
    right_decoder_->flush();
}

void stereo_surface_decoder::stop() {
    if (left_decoder_) left_decoder_->stop();
    if (right_decoder_) right_decoder_->stop();
}
