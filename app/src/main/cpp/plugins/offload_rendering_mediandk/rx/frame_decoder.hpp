#pragma once

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>

#include <jni.h>
#include <media/NdkMediaCodec.h>
#include <queue>
#include <thread>
#include <vector>

namespace ILLIXR {

// Surface-based frame decoder for a single eye using Android MediaCodec.
// Outputs to SurfaceTexture for zero-copy GPU access.
class frame_decoder {
public:
    frame_decoder(int eye_index, int width, int height);
    ~frame_decoder();

    // Non-copyable, non-movable
    frame_decoder(const frame_decoder&) = delete;
    frame_decoder& operator=(const frame_decoder&) = delete;
    frame_decoder(frame_decoder&&) = delete;
    frame_decoder& operator=(frame_decoder&&) = delete;

    // Initialize the decoder.
    // Must be called from a thread with JNI access and with EGL context current.
    bool initialize(JNIEnv* env, EGLDisplay egl_display, EGLContext egl_context);

    // Queue encoded H.264 data for decoding.
    // Thread-safe, can be called from network thread.
    bool queue_encoded_data(const uint8_t* data, size_t size,
                            int64_t timestamp_us, bool is_keyframe);

    // Update the texture with the latest decoded frame.
    // Must be called from the GL render thread with EGL context current.
    bool update_texture();

    // Get the external OES texture ID.
    // Valid after initialize() returns true.
    [[nodiscard]] GLuint get_texture_id() const { return external_texture_; }

    // Get the texture transform matrix.
    void get_transform_matrix(float* matrix);

    // Get the latest timestamp of decoded frame
    [[nodiscard]] int64_t get_latest_timestamp_us() const {
        return latest_timestamp_us_.load();
    }

    // Check if decoder is ready
    [[nodiscard]] bool is_ready() const { return initialized_.load(); }

    // Flush decoder (on stream discontinuity)
    void flush();

    // Stop decoder and release resources
    void stop();

private:
    void decode_loop();
    bool create_surface_texture(JNIEnv* env);
    void release_surface_texture(JNIEnv* env);
    bool configure_codec();

    int eye_index_;
    int width_;
    int height_;

    // EGL references
    EGLDisplay egl_display_{EGL_NO_DISPLAY};
    EGLContext egl_context_{EGL_NO_CONTEXT};

    // MediaCodec
    AMediaCodec* codec_{nullptr};
    ANativeWindow* native_window_{nullptr};

    // SurfaceTexture (JNI)
    JavaVM* jvm_{nullptr};
    jobject surface_texture_{nullptr};
    jobject surface_{nullptr};
    jmethodID update_tex_image_method_{nullptr};
    jmethodID get_transform_matrix_method_{nullptr};

    // GL texture
    GLuint external_texture_{0};

    // Threading
    std::thread decode_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};

    // Input queue
    struct encoded_packet {
        std::vector<uint8_t> data;
        int64_t timestamp_us{0};
        bool is_keyframe{false};
    };
    std::queue<encoded_packet> input_queue_;
    std::mutex input_mutex_;
    std::condition_variable input_cv_;

    // Frame state
    std::atomic<bool> frame_available_{false};
    std::atomic<int64_t> latest_timestamp_us_{0};
    std::atomic<uint64_t> frames_decoded_{0};
};

}
