#pragma once

#include <queue>
#include <media/NdkMediaCodec.h>
#include <thread>

namespace ILLIXR {
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
};

struct DecodedFrame {
    std::vector<uint8_t> data;  // NV12 format data
    int width;
    int height;
    int64_t timestamp;

    // Helper to get Y plane pointer
    [[nodiscard]] const uint8_t* get_y_plane() const {
        return data.data();
    }

    // Helper to get UV plane pointer
    [[nodiscard]] const uint8_t* get_uv_plane() const {
        return data.data() + (width * height);
    }

    // Convert NV12 to RGBA for OpenXR rendering
    [[nodiscard]] std::vector<uint8_t> to_rgba() const;

    // Convert NV12 to RGB (no alpha) for OpenXR
    [[nodiscard]] std::vector<uint8_t> to_rgb() const;
};


class frame_decoder {
public:
    frame_decoder(int w, int h);

    ~frame_decoder();

    void stop();

    void queue_frame(const std::vector <uint8_t>& encoded_data);

    void queue_frame(const uint8_t* data, size_t size);

    void flush();

    bool get_decoded_frame(DecodedFrame& frame, int timeout_mil = -1);

    bool has_decoded_frame();

    size_t get_decoded_frame_count();

    int get_width() const { return width_; }

    int get_height() const { return height_; }

private:
    void decode_loop();
    bool config_sent_ = false;  // Track if we've sent SPS/PPS to codec
    std::vector<uint8_t> cached_sps_;
    std::vector<uint8_t> cached_pps_;
    AMediaCodec* codec_ = nullptr;
    int width_, height_;
    bool running_ = false;
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
    std::thread decode_thread_;
    std::queue <std::vector<uint8_t>> input_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<DecodedFrame> output_queue_;
    std::mutex output_mutex_;
    std::condition_variable output_cv_;
    size_t max_output_queue_ = 30;
};

}
