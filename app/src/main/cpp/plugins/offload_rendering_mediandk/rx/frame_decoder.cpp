#include "frame_decoder.hpp"

#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <cstring>
#include <media/NdkMediaFormat.h>
#include <spdlog/spdlog.h>

using namespace ILLIXR;

// Helper to attach/detach JNI on decoder thread
class jni_thread_attacher {
public:
    explicit jni_thread_attacher(JavaVM* jvm) : jvm_(jvm), env_(nullptr), attached_(false) {
        if (jvm_) {
            jint result = jvm_->GetEnv(reinterpret_cast<void**>(&env_), JNI_VERSION_1_6);
            if (result == JNI_EDETACHED) {
                if (jvm_->AttachCurrentThread(&env_, nullptr) == JNI_OK) {
                    attached_ = true;
                }
            }
        }
    }

    ~jni_thread_attacher() {
        if (attached_ && jvm_) {
            jvm_->DetachCurrentThread();
        }
    }

    [[nodiscard]] JNIEnv* get_env() const { return env_; }
    [[nodiscard]] bool is_attached() const { return env_ != nullptr; }

private:
    JavaVM* jvm_;
    JNIEnv* env_;
    bool attached_;
};

frame_decoder::frame_decoder(int eye_index, int width, int height)
        : eye_index_(eye_index)
        , width_(width)
        , height_(height) {}

frame_decoder::~frame_decoder() {
    stop();
}

bool frame_decoder::initialize(JNIEnv* env, EGLDisplay egl_display, EGLContext egl_context) {
    if (initialized_.load()) {
        spdlog::get("illixr")->warn("frame_decoder[{}] already initialized", eye_index_);
        return true;
    }

    egl_display_ = egl_display;
    egl_context_ = egl_context;

    // Get JavaVM for later thread attachment
    if (env->GetJavaVM(&jvm_) != JNI_OK) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to get JavaVM", eye_index_);
        return false;
    }

    // Verify GL context is current
    if (eglGetCurrentContext() == EGL_NO_CONTEXT) {
        spdlog::get("illixr")->error("frame_decoder[{}]: No EGL context during init", eye_index_);
        return false;
    }

    // Create external OES texture
    glGenTextures(1, &external_texture_);
    if (external_texture_ == 0) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to generate texture", eye_index_);
        return false;
    }

    glBindTexture(GL_TEXTURE_EXTERNAL_OES, external_texture_);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);

    GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR) {
        spdlog::get("illixr")->error("frame_decoder[{}]: GL error after texture creation: 0x{:X}",
                      eye_index_, gl_error);
        glDeleteTextures(1, &external_texture_);
        external_texture_ = 0;
        return false;
    }

    // Create SurfaceTexture via JNI
    if (!create_surface_texture(env)) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to create SurfaceTexture", eye_index_);
        glDeleteTextures(1, &external_texture_);
        external_texture_ = 0;
        return false;
    }

    // Configure MediaCodec
    if (!configure_codec()) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to configure codec", eye_index_);
        release_surface_texture(env);
        glDeleteTextures(1, &external_texture_);
        external_texture_ = 0;
        return false;
    }

    // Start decode thread
    running_.store(true);
    decode_thread_ = std::thread(&frame_decoder::decode_loop, this);

    initialized_.store(true);
    spdlog::get("illixr")->info("frame_decoder[{}]: Initialized {}x{}, texture={}",
                 eye_index_, width_, height_, external_texture_);
    return true;
}

bool frame_decoder::create_surface_texture(JNIEnv* env) {
    // Find SurfaceTexture class
    jclass surface_texture_class = env->FindClass("android/graphics/SurfaceTexture");
    if (!surface_texture_class) {
        spdlog::get("illixr")->error("frame_decoder[{}]: SurfaceTexture class not found", eye_index_);
        return false;
    }

    // Get constructor(int texName)
    jmethodID constructor = env->GetMethodID(surface_texture_class, "<init>", "(I)V");
    if (!constructor) {
        spdlog::get("illixr")->error("frame_decoder[{}]: SurfaceTexture constructor not found", eye_index_);
        env->DeleteLocalRef(surface_texture_class);
        return false;
    }

    // Create SurfaceTexture
    jobject local_st = env->NewObject(surface_texture_class, constructor,
                                      static_cast<jint>(external_texture_));
    if (!local_st || env->ExceptionCheck()) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to create SurfaceTexture", eye_index_);
        env->ExceptionClear();
        env->DeleteLocalRef(surface_texture_class);
        return false;
    }

    surface_texture_ = env->NewGlobalRef(local_st);
    env->DeleteLocalRef(local_st);

    // Cache method IDs
    update_tex_image_method_ = env->GetMethodID(surface_texture_class, "updateTexImage", "()V");
    get_transform_matrix_method_ = env->GetMethodID(surface_texture_class,
                                                    "getTransformMatrix", "([F)V");

    if (!update_tex_image_method_ || !get_transform_matrix_method_) {
        spdlog::get("illixr")->error("frame_decoder[{}]: SurfaceTexture methods not found", eye_index_);
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
        env->DeleteLocalRef(surface_texture_class);
        return false;
    }

    // Set default buffer size
    jmethodID set_buffer_size = env->GetMethodID(surface_texture_class,
                                                 "setDefaultBufferSize", "(II)V");
    if (set_buffer_size) {
        env->CallVoidMethod(surface_texture_, set_buffer_size, width_, height_);
    }

    env->DeleteLocalRef(surface_texture_class);

    // Create Surface from SurfaceTexture
    jclass surface_class = env->FindClass("android/view/Surface");
    if (!surface_class) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Surface class not found", eye_index_);
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
        return false;
    }

    jmethodID surface_ctor = env->GetMethodID(surface_class, "<init>",
                                              "(Landroid/graphics/SurfaceTexture;)V");
    if (!surface_ctor) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Surface constructor not found", eye_index_);
        env->DeleteLocalRef(surface_class);
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
        return false;
    }

    jobject local_surface = env->NewObject(surface_class, surface_ctor, surface_texture_);
    if (!local_surface || env->ExceptionCheck()) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to create Surface", eye_index_);
        env->ExceptionClear();
        env->DeleteLocalRef(surface_class);
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
        return false;
    }

    surface_ = env->NewGlobalRef(local_surface);
    env->DeleteLocalRef(local_surface);
    env->DeleteLocalRef(surface_class);

    // Get ANativeWindow
    native_window_ = ANativeWindow_fromSurface(env, surface_);
    if (!native_window_) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to get ANativeWindow", eye_index_);
        env->DeleteGlobalRef(surface_);
        surface_ = nullptr;
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
        return false;
    }

    return true;
}

void frame_decoder::release_surface_texture(JNIEnv* env) {
    if (native_window_) {
        ANativeWindow_release(native_window_);
        native_window_ = nullptr;
    }

    if (surface_ && env) {
        jclass cls = env->GetObjectClass(surface_);
        if (cls) {
            jmethodID release = env->GetMethodID(cls, "release", "()V");
            if (release) {
                env->CallVoidMethod(surface_, release);
            }
            env->DeleteLocalRef(cls);
        }
        env->DeleteGlobalRef(surface_);
        surface_ = nullptr;
    }

    if (surface_texture_ && env) {
        jclass cls = env->GetObjectClass(surface_texture_);
        if (cls) {
            jmethodID release = env->GetMethodID(cls, "release", "()V");
            if (release) {
                env->CallVoidMethod(surface_texture_, release);
            }
            env->DeleteLocalRef(cls);
        }
        env->DeleteGlobalRef(surface_texture_);
        surface_texture_ = nullptr;
    }
}

bool frame_decoder::configure_codec() {
    // Try Qualcomm hardware decoder (Quest 3)
    codec_ = AMediaCodec_createCodecByName("OMX.qcom.video.decoder.hevc");
    if (!codec_) {
        codec_ = AMediaCodec_createDecoderByType("c2.qti.hevc.decoder");
    }
    if (!codec_) {
        codec_ = AMediaCodec_createDecoderByType("video/hevc");
    }
    if (!codec_) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to create decoder", eye_index_);
        return false;
    }

    AMediaFormat* format = AMediaFormat_new();
    AMediaFormat_setString(format, AMEDIAFORMAT_KEY_MIME, "video/hevc");  // h265
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_WIDTH, width_);
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_HEIGHT, height_);
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_MAX_INPUT_SIZE, width_ * height_);

    // Low latency hints
    AMediaFormat_setInt32(format, "low-latency", 1);
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_PRIORITY, 0);  // Realtime

    // Configure with surface output
    media_status_t status = AMediaCodec_configure(codec_, format, native_window_, nullptr, 0);
    AMediaFormat_delete(format);

    if (status != AMEDIA_OK) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Configure failed: {}",
                                     eye_index_, static_cast<int>(status));
        AMediaCodec_delete(codec_);
        codec_ = nullptr;
        return false;
    }

    status = AMediaCodec_start(codec_);
    if (status != AMEDIA_OK) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Start failed: {}",
                      eye_index_, static_cast<int>(status));
        AMediaCodec_delete(codec_);
        codec_ = nullptr;
        return false;
    }

    return true;
}

bool frame_decoder::queue_encoded_data(const uint8_t* data, size_t size,
                                               int64_t timestamp_us, bool is_keyframe) {
    if (!running_.load()) {
        return false;
    }

    encoded_packet packet;
    packet.data.assign(data, data + size);
    packet.timestamp_us = timestamp_us;
    packet.is_keyframe = is_keyframe;

    {
        std::lock_guard<std::mutex> lock(input_mutex_);

        // Latency control: drop old P-frames if queue is deep
        constexpr size_t max_queue = 5;
        while (input_queue_.size() >= max_queue) {
            if (!input_queue_.front().is_keyframe) {
                input_queue_.pop();
            } else {
                break;
            }
        }

        input_queue_.push(std::move(packet));
    }
    input_cv_.notify_one();

    return true;
}

void frame_decoder::decode_loop() {
    jni_thread_attacher jni(jvm_);
    if (!jni.is_attached()) {
        spdlog::get("illixr")->error("frame_decoder[{}]: Failed to attach to JVM", eye_index_);
        return;
    }

    spdlog::get("illixr")->info("frame_decoder[{}]: Decode loop started", eye_index_);

    while (running_.load()) {
        // Feed input
        {
            std::unique_lock<std::mutex> lock(input_mutex_);
            if (input_queue_.empty()) {
                input_cv_.wait_for(lock, std::chrono::milliseconds(10));
            }

            int fed = 0;
            while (!input_queue_.empty() && fed < 3) {
                ssize_t idx = AMediaCodec_dequeueInputBuffer(codec_, 0);
                if (idx < 0)
                    break;

                auto& pkt = input_queue_.front();

                size_t buf_size = 0;
                uint8_t* buf = AMediaCodec_getInputBuffer(codec_, idx, &buf_size);

                if (buf && buf_size >= pkt.data.size()) {
                    std::memcpy(buf, pkt.data.data(), pkt.data.size());

                    uint32_t flags = pkt.is_keyframe ? AMEDIACODEC_BUFFER_FLAG_KEY_FRAME : 0;
                    AMediaCodec_queueInputBuffer(codec_, idx, 0, pkt.data.size(),
                                                 pkt.timestamp_us, flags);
                    fed++;
                }

                input_queue_.pop();
            }
        }

        // Drain output (render to surface)
        AMediaCodecBufferInfo info;
        ssize_t idx;

        while ((idx = AMediaCodec_dequeueOutputBuffer(codec_, &info, 0)) >= 0) {
            // Release with render=true to send to SurfaceTexture
            media_status_t status = AMediaCodec_releaseOutputBuffer(codec_, idx, true);

            if (status == AMEDIA_OK) {
                frame_available_.store(true);
                latest_timestamp_us_.store(info.presentationTimeUs);
                frames_decoded_.fetch_add(1);
            }
        }

        if (idx == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    spdlog::get("illixr")->info("frame_decoder[{}]: Decode loop stopped, decoded {} frames",
                 eye_index_, frames_decoded_.load());
}

bool frame_decoder::update_texture() {
    if (!initialized_.load() || !surface_texture_ || !jvm_) {
        return false;
    }

    if (!frame_available_.exchange(false)) {
        return false;
    }

    jni_thread_attacher jni(jvm_);
    JNIEnv* env = jni.get_env();
    if (!env) {
        return false;
    }

    // Check if we have a GL context - if not, try to make ours current
    if (eglGetCurrentContext() == EGL_NO_CONTEXT) {
        if (egl_display_ != EGL_NO_DISPLAY && egl_context_ != EGL_NO_CONTEXT) {
            if (!eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, egl_context_)) {
                spdlog::get("illixr")->error("surface_frame_decoder[{}]: Failed to make context current in update_texture", eye_index_);
                return false;
            }
        } else {
            spdlog::get("illixr")->error("surface_frame_decoder[{}]: No GL context in update_texture", eye_index_);
            return false;
        }
    }

    env->CallVoidMethod(surface_texture_, update_tex_image_method_);

    if (env->ExceptionCheck()) {
        env->ExceptionClear();
        return false;
    }

    return true;
}

void frame_decoder::get_transform_matrix(float* matrix) {
    // Default to identity
    std::memset(matrix, 0, 16 * sizeof(float));
    matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1.0f;

    if (!initialized_.load() || !surface_texture_ || !jvm_) {
        return;
    }

    jni_thread_attacher jni(jvm_);
    JNIEnv* env = jni.get_env();
    if (!env)
        return;

    jfloatArray arr = env->NewFloatArray(16);
    if (!arr)
        return;

    env->CallVoidMethod(surface_texture_, get_transform_matrix_method_, arr);

    if (!env->ExceptionCheck()) {
        env->GetFloatArrayRegion(arr, 0, 16, matrix);
    } else {
        env->ExceptionClear();
    }

    env->DeleteLocalRef(arr);
}

void frame_decoder::flush() {
    if (!codec_)
        return;

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        while (!input_queue_.empty())
            input_queue_.pop();
    }

    AMediaCodec_flush(codec_);
    frame_available_.store(false);
}

void frame_decoder::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    input_cv_.notify_all();

    if (decode_thread_.joinable()) {
        decode_thread_.join();
    }

    if (codec_) {
        AMediaCodec_stop(codec_);
        AMediaCodec_delete(codec_);
        codec_ = nullptr;
    }

    if (jvm_) {
        jni_thread_attacher jni(jvm_);
        release_surface_texture(jni.get_env());
    }

    if (external_texture_ != 0) {
        glDeleteTextures(1, &external_texture_);
        external_texture_ = 0;
    }

    initialized_.store(false);
    spdlog::get("illixr")->info("frame_decoder[{}]: Stopped", eye_index_);
}


