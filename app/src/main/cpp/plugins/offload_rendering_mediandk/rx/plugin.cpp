#include "plugin.hpp"

#include "illixr/egl/egl_context_manager.hpp"

#include "jni_helper.hpp"
using namespace ILLIXR;
using namespace ILLIXR::data_format;

[[maybe_unused]] android_media_decoder::android_media_decoder(const std::string& name, phonebook* pb)
        : threadloop{name, pb}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , clock_{pb->lookup_impl<relative_clock>()}
        , frame_writer_{switchboard_->get_writer<data_format::dual_frames>("unity_rendered_frame")}
        , compressed_frame_reader_{switchboard_->get_buffered_reader<switchboard::event_wrapper<std::string>>("compressed_frame")}
        , app_{switchboard_->get_android_app()} {}

android_media_decoder::~android_media_decoder() {
    if (decoder_) {
        decoder_->stop();
    }

    // Release our thread's EGL context
    egl_context_manager::instance().release_current_thread_context();
}

void android_media_decoder::_p_thread_setup() {
    spdlog::get("illixr")->info("android_media_decoder: Thread setup starting");

    // Wait for OpenXR plugin to register the primary context
    if (!egl_context_manager::instance().wait_for_initialization(10000)) {
        spdlog::get("illixr")->error("android_media_decoder: Timeout waiting for EGL context");
        return;
    }

    spdlog::get("illixr")->info("android_media_decoder: EGL context manager ready");

    // Get/create a shared context for this thread
    if (!egl_context_manager::instance().make_current_for_thread()) {
        spdlog::get("illixr")->error("android_media_decoder: Failed to get GL context");
        return;
    }

    spdlog::get("illixr")->info("android_media_decoder: GL context acquired: {}",
                                (void*)eglGetCurrentContext());

    // Now we can initialize the decoder
    JNIEnv* env = get_jni_env(app_);
    if (!env) {
        spdlog::get("illixr")->error("android_media_decoder: Failed to get JNI env");
        return;
    }

    auto& ctx_mgr = egl_context_manager::instance();

    decoder_ = std::make_unique<stereo_surface_decoder>(HEADSET_WIDTH, HEADSET_HEIGHT);
    if (!decoder_->initialize(env, ctx_mgr.get_display(), eglGetCurrentContext())) {
        spdlog::get("illixr")->error("android_media_decoder: Failed to initialize decoder");
        decoder_.reset();
        return;
    }

    decoder_initialized_ = true;
    spdlog::get("illixr")->info("android_media_decoder: Decoder initialized successfully");
}

void android_media_decoder::_p_one_iteration() {
    if (!decoder_initialized_) {
        return;
    }
    // Ensure context is current (should already be, but just in case)
    if (eglGetCurrentContext() == EGL_NO_CONTEXT) {
        egl_context_manager::instance().make_current_for_thread();
    }
    if (compressed_frame_reader_.size() > 0) {
        auto buffer_ptr = compressed_frame_reader_.dequeue();
        buffer_str_ = **buffer_ptr;
        std::string::size_type end_position = buffer_str_.find(delimiter_);
        rendered_frame_proto::CompressedFrame frame;
        if (frame.ParseFromString(buffer_str_.substr(0, end_position))) {
            spdlog::get("illixr")->debug("Got frame {} {}", frame.left_eye_size(), frame.right_eye_size());
            decompress_frame(frame);
        } else {
            spdlog::get("illixr")->error("Error parsing");
        }
    }
    send_frame();
}

void android_media_decoder::decompress_frame(const rendered_frame_proto::CompressedFrame& frame) {
    decoder_->queue_encoded_data(0, reinterpret_cast<const uint8_t*>(frame.left_eye().data()),
                                 frame.left_eye_size(), static_cast<long>(frame.timestamp()), true);
    decoder_->queue_encoded_data(1, reinterpret_cast<const uint8_t*>(frame.right_eye().data()),
                                 frame.right_eye_size(), static_cast<long>(frame.timestamp()), true);
}

void android_media_decoder::send_frame() {
    if (!decoder_ || !decoder_->is_ready())
        return;

    auto now = clock_->now();
    dual_frames frame = decoder_->get_current_frame(now);

    if (frame.is_valid()) {
        frame_writer_.put(frame_writer_.allocate(std::move(frame)));
    }

}

PLUGIN_MAIN(android_media_decoder)
