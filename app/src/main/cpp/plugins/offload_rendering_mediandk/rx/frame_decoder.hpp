#pragma once

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>

#include <media/NdkMediaCodec.h>
#include <jni.h>

#include <queue>
#include <thread>
#include <vector>

namespace ILLIXR {

/// Surface-based frame decoder for a single eye using Android MediaCodec.
/// Outputs to SurfaceTexture for zero-copy GPU access.
class frame_decoder {
public:
    frame_decoder(int eye_index, int width, int height);
    ~frame_decoder();

    // Non-copyable, non-movable
    frame_decoder(const frame_decoder&) = delete;
    frame_decoder& operator=(const frame_decoder&) = delete;
    frame_decoder(frame_decoder&&) = delete;
    frame_decoder& operator=(frame_decoder&&) = delete;

    /// Initialize the decoder.
    /// Must be called from a thread with JNI access and with EGL context current.
    /// @param env JNI environment
    /// @param egl_display EGL display
    /// @param egl_context EGL context (decoder will use this context's share group)
    /// @return true if initialization succeeded
    bool initialize(JNIEnv* env, EGLDisplay egl_display, EGLContext egl_context);

    /// Queue encoded H.264 data for decoding.
    /// Thread-safe, can be called from network thread.
    /// @param data Encoded NAL unit (Annex B format)
    /// @param size Size in bytes
    /// @param timestamp_us Presentation timestamp in microseconds
    /// @param is_keyframe True if this is an IDR frame
    /// @return true if queued successfully
    bool queue_encoded_data(const uint8_t* data, size_t size,
                            int64_t timestamp_us, bool is_keyframe);

    /// Update the texture with the latest decoded frame.
    /// Must be called from the GL render thread with EGL context current.
    /// @return true if a new frame was available
    bool update_texture();

    /// Get the external OES texture ID.
    /// Valid after initialize() returns true.
    [[nodiscard]] GLuint get_texture_id() const { return external_texture_; }

    /// Get the texture transform matrix.
    /// @param matrix Output array for 4x4 column-major matrix
    void get_transform_matrix(float* matrix);

    /// Get the latest timestamp of decoded frame
    [[nodiscard]] int64_t get_latest_timestamp_us() const {
        return latest_timestamp_us_.load();
    }

    /// Check if decoder is ready
    [[nodiscard]] bool is_ready() const { return initialized_.load(); }

    /// Flush decoder (on stream discontinuity)
    void flush();

    /// Stop decoder and release resources
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
        int64_t timestamp_us;
        bool is_keyframe;
    };
    std::queue<encoded_packet> input_queue_;
    std::mutex input_mutex_;
    std::condition_variable input_cv_;

    // Frame state
    std::atomic<bool> frame_available_{false};
    std::atomic<int64_t> latest_timestamp_us_{0};
    std::atomic<uint64_t> frames_decoded_{0};
};

/*
// H.264 NAL Unit Types
enum H264NalUnitType {
    NAL_UNIT_TYPE_UNSPECIFIED = 0,
    NAL_UNIT_TYPE_CODED_SLICE_NON_IDR = 1,
    NAL_UNIT_TYPE_CODED_SLICE_PARTITION_A = 2,
    NAL_UNIT_TYPE_CODED_SLICE_PARTITION_B = 3,
    NAL_UNIT_TYPE_CODED_SLICE_PARTITION_C = 4,
    NAL_UNIT_TYPE_CODED_SLICE_IDR = 5,
    NAL_UNIT_TYPE_SEI = 6,
    NAL_UNIT_TYPE_SPS = 7,  // Sequence Parameter Set - REQUIRED
    NAL_UNIT_TYPE_PPS = 8,  // Picture Parameter Set - REQUIRED
    NAL_UNIT_TYPE_AUD = 9,  // Access Unit Delimiter
    NAL_UNIT_TYPE_END_SEQUENCE = 10,
    NAL_UNIT_TYPE_END_STREAM = 11,
    NAL_UNIT_TYPE_FILLER_DATA = 12,
};

// H.264 Format Detection

enum H264Format {
    H264_FORMAT_UNKNOWN,
    H264_FORMAT_ANNEX_B,      // Start codes: 0x00 0x00 0x00 0x01
    H264_FORMAT_AVCC          // Length-prefixed (MP4 style)
};*/

    /*void stop();

    void queue_frame(const std::vector <uint8_t>& encoded_data);

    void queue_frame(const uint8_t* data, size_t size);

    void flush();

    bool get_decoded_frame(DecodedFrame& frame, int timeout_mil = -1);

    bool has_decoded_frame();

    size_t get_decoded_frame_count();

    int get_width() const { return width_; }

    int get_height() const { return height_; }
     */


    /*bool config_sent_ = false;  // Track if we've sent SPS/PPS to codec
    std::vector<uint8_t> cached_sps_;
    std::vector<uint8_t> cached_pps_;
    static bool has_b_frames(const uint8_t* data, size_t size);
    static AMediaCodec* create_decoder_with_fallback(int width, int height);
    void check_first_frame_compatibility(const std::vector<uint8_t>& data);
    static uint8_t get_nal_unit_type(uint8_t nal_header);
    static H264Format detect_h264_format(const uint8_t* data, size_t size);
    static std::vector<uint8_t> convert_avcc_to_annexb(const uint8_t* data, size_t size);
    static void check_and_convert_h264_format(std::vector<uint8_t>& encoded_data);
    static void analyze_h264_data(const uint8_t* data, size_t size, const char* label);
    static bool has_sps_pps(const uint8_t* data, size_t size);
    static bool extract_sps_pps(const uint8_t* data, size_t size,
                                               std::vector<uint8_t>& sps_out,
                                               std::vector<uint8_t>& pps_out);
    std::queue <std::vector<uint8_t>> input_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<DecodedFrame> output_queue_;
    std::mutex output_mutex_;
    std::condition_variable output_cv_;
    size_t max_output_queue_ = 30;*/

}
