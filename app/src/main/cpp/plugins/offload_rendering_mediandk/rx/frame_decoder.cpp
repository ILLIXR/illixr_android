#include "frame_decoder.hpp"

using namespace ILLIXR;

#include <cstdint>
#include <media/NdkMediaFormat.h>
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <vector>




// Quest 3 MediaCodec "Unsupported input buffer" Troubleshooting

/*
 * COMMON CAUSES OF "Unsupported input buffer" ON QUEST 3:
 *
 * 1. Setting color format during configuration (REMOVED in fix above)
 * 2. Wrong H.264 profile/level in encoder
 * 3. Resolution mismatch between config and actual data
 * 4. Missing or corrupt SPS/PPS headers
 * 5. B-frames in H.264 stream (Quest 3 hardware decoder limitation)
 */

// ============================================================================
// SOLUTION 1: Check Your Encoder Settings (NVIDIA/Source)
// ============================================================================

/*
 * Make sure your NVIDIA encoder (or whatever is encoding) uses these settings:
 *
 * Profile: Baseline or Main (NOT High)
 * Level: 4.0 or 4.1 (supports up to 1080p60)
 * B-frames: 0 (CRITICAL - Quest 3 may not support B-frames)
 * GOP Structure: I and P frames only
 * Bitrate: 5-15 Mbps for 2064x2208
 * Frame Rate: 72 or 90 fps (Quest 3 native)
 *
 * For NVIDIA encoding:
 * - NV_ENC_CODEC_H264_GUID
 * - profileGUID = NV_ENC_H264_PROFILE_BASELINE_GUID or MAIN
 * - gopLength = 90 (1 second at 90fps)
 * - numBFrames = 0  // CRITICAL
 * - idrPeriod = 90  // Keyframe every second
 */

// ============================================================================
// SOLUTION 2: Verify H.264 Stream Compatibility
// ============================================================================

// Save first few frames to file for analysis with ffprobe
void save_h264_for_analysis(const uint8_t* data, size_t size, int frame_num) {
    return;
    if (frame_num < 3) {  // Save first 3 frames
        char filename[64];
        snprintf(filename, sizeof(filename), "/sdcard/frame_%d.h264", frame_num);
        FILE* f = fopen(filename, "wb");
        if (f) {
            fwrite(data, 1, size, f);
            fclose(f);
            spdlog::get("illixr")->info("Saved frame {} to {}", frame_num, filename);
            spdlog::get("illixr")->info("Analyze with: ffprobe {}", filename);
        } else {
            spdlog::get("illixr")->error("Could not open file");
        }
    }
}
// Add this function to check for B-frames
bool frame_decoder::has_b_frames(const uint8_t* data, size_t size) {
    // B-frames are NAL type 0 or 1 with specific slice types
    for (size_t i = 0; i < size - 5; i++) {
        if ((data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x00 && data[i+3] == 0x01) ||
            (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01)) {

            size_t offset = (data[i+2] == 0x01) ? 3 : 4;
            uint8_t nal_type = data[i + offset] & 0x1F;

            // NAL type 1 (non-IDR slice) could be B-frame
            if (nal_type == 1) {
                // Check slice type (first byte after NAL header)
                uint8_t slice_header = data[i + offset + 1];
                uint8_t slice_type = slice_header & 0x1F;

                // Slice types: 0=P, 1=B, 2=I, 5=P, 6=B, 7=I
                if (slice_type == 1 || slice_type == 6) {
                    return true;  // B-frame detected!
                }
            }
        }
    }
    return false;
}

// ============================================================================
// SOLUTION 3: Try Software Decoder as Fallback
// ============================================================================

AMediaCodec* frame_decoder::create_decoder_with_fallback(int width, int height) {
    AMediaCodec* codec = nullptr;

    // Try hardware decoders in order of preference
    const char* codec_names[] = {
            "OMX.qcom.video.decoder.avc",     // Qualcomm hardware (Quest 3)
            "c2.qti.avc.decoder",              // Codec2 Qualcomm
            "c2.android.avc.decoder",          // Android software fallback
            nullptr
    };

    for (int i = 0; codec_names[i] != nullptr; i++) {
        codec = AMediaCodec_createCodecByName(codec_names[i]);
        if (codec) {
            spdlog::get("illixr")->info("Using decoder: {}", codec_names[i]);
            return codec;
        }
    }

    // Last resort: let system choose
    codec = AMediaCodec_createDecoderByType("video/avc");
    if (codec) {
        spdlog::get("illixr")->info("Using system default H.264 decoder");
    }

    return codec;
}


// ============================================================================
// SOLUTION 5: Add B-frame Detection to decode_loop
// ============================================================================

// In your decode_loop, add this check on first frame:
void frame_decoder::check_first_frame_compatibility(const std::vector<uint8_t>& data) {
    static bool checked = false;
    if (checked) return;
    checked = true;

    spdlog::get("illixr")->info("=== FIRST FRAME DIAGNOSTICS ===");
    spdlog::get("illixr")->info("Frame size: {} bytes", data.size());

    // Check for B-frames
    if (has_b_frames(data.data(), data.size())) {
        spdlog::get("illixr")->error("!!! B-FRAMES DETECTED !!!");
        spdlog::get("illixr")->error("Quest 3 hardware decoder may not support B-frames!");
        spdlog::get("illixr")->error("Configure your encoder with: numBFrames = 0");
    } else {
        spdlog::get("illixr")->info("Good: No B-frames detected");
    }
    // Check format
    if (data.size() >= 4) {
        if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x01) {
            spdlog::get("illixr")->info("Format: Annex B (4-byte start code) ✓");
        } else if (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x01) {
            spdlog::get("illixr")->info("Format: Annex B (3-byte start code) ✓");
        } else {
            spdlog::get("illixr")->warn("Format: Unknown (first bytes: %02X %02X %02X %02X)",
                                        data[0], data[1], data[2], data[3]);
        }
    }

    // Check for SPS/PPS
    bool has_sps = false;
    bool has_pps = false;
    bool has_idr = false;

    for (size_t i = 0; i < data.size() - 5; i++) {
        if ((data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x00 && data[i+3] == 0x01) ||
            (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01)) {

            size_t offset = (data[i+2] == 0x01) ? 3 : 4;
            uint8_t nal_type = (data[i + offset] & 0x1F);

            if (nal_type == 7) has_sps = true;
            if (nal_type == 8) has_pps = true;
            if (nal_type == 5) has_idr = true;
        }
    }

    spdlog::get("illixr")->info("SPS present: {} {}", has_sps, has_sps ? "✓" : "✗ MISSING!");
    spdlog::get("illixr")->info("PPS present: {} {}", has_pps, has_pps ? "✓" : "✗ MISSING!");
    spdlog::get("illixr")->info("IDR present: {} {}", has_idr, has_idr ? "✓" : "✗");

    if (!has_sps || !has_pps) {
        spdlog::get("illixr")->error("CRITICAL: Missing required headers!");
        spdlog::get("illixr")->error("Decoder will likely fail with 'Unsupported input buffer'");
    }

    spdlog::get("illixr")->info("================================");
}

// ============================================================================
// QUICK FIX CHECKLIST:
// ============================================================================

/*
 * [X] Remove color format from AMediaFormat configuration (DONE ABOVE)
 * [X] Verify encoder outputs Baseline or Main profile (not High)
 * [ ] Verify encoder has numBFrames = 0
 * [ ] Verify resolution matches: encoder = 2064x2208, decoder = 2064x2208
 * [X] Verify first frame contains SPS + PPS + IDR
 * [ ] Check Android logcat for more specific errors:
 *     adb logcat | grep -i "codec\|media\|omx"
 */






// Get NAL unit type from the byte following start code
uint8_t frame_decoder::get_nal_unit_type(uint8_t nal_header) {
    // NAL unit type is in bits 0-4 of the NAL header byte
    return nal_header & 0x1F;
}

// Detect H.264 format
H264Format frame_decoder::detect_h264_format(const uint8_t* data, size_t size) {
    if (!data || size < 4) {
        return H264_FORMAT_UNKNOWN;
    }

    // Check for Annex B start codes
    if (data[0] == 0x00 && data[1] == 0x00) {
        if (data[2] == 0x00 && data[3] == 0x01) {
            return H264_FORMAT_ANNEX_B;  // 4-byte start code
        }
        if (data[2] == 0x01) {
            return H264_FORMAT_ANNEX_B;  // 3-byte start code
        }
    }

    // Check for AVCC format (first 4 bytes are NAL unit length)
    uint32_t nal_length = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];

    // Sanity check: NAL length should be reasonable (< total size - 4)
    if (nal_length > 0 && nal_length < size - 4) {
        // Check if there's a valid NAL unit type after the length
        uint8_t nal_type = get_nal_unit_type(data[4]);
        if (nal_type >= 1 && nal_type <= 23) {  // Valid NAL unit types
            return H264_FORMAT_AVCC;
        }
    }

    return H264_FORMAT_UNKNOWN;
}

// Convert AVCC to Annex B format (MediaCodec needs Annex B)
std::vector<uint8_t> frame_decoder::convert_avcc_to_annexb(const uint8_t* data, size_t size) {
    std::vector<uint8_t> annexb_data;
    annexb_data.reserve(size + 100);  // Extra space for start codes

    size_t offset = 0;
    while (offset < size - 4) {
        // Read NAL unit length (4 bytes, big-endian)
        uint32_t nal_length = (data[offset] << 24) |
                              (data[offset + 1] << 16) |
                              (data[offset + 2] << 8) |
                              data[offset + 3];
        offset += 4;

        if (nal_length == 0 || offset + nal_length > size) {
            spdlog::get("illixr")->error("Invalid NAL length: {} at offset {}", nal_length, offset - 4);
            break;
        }

        // Add Annex B start code
        annexb_data.push_back(0x00);
        annexb_data.push_back(0x00);
        annexb_data.push_back(0x00);
        annexb_data.push_back(0x01);

        // Copy NAL unit data
        annexb_data.insert(annexb_data.end(), data + offset, data + offset + nal_length);

        offset += nal_length;
    }

    spdlog::get("illixr")->info("Converted AVCC to Annex B: {} bytes -> {} bytes",
                                size, annexb_data.size());

    return annexb_data;
}

// Complete format checker with conversion
void frame_decoder::check_and_convert_h264_format(std::vector<uint8_t>& encoded_data) {
    static bool format_detected = false;
    static H264Format detected_format = H264_FORMAT_UNKNOWN;

    if (!format_detected) {
        detected_format = detect_h264_format(encoded_data.data(), encoded_data.size());

        const char* format_name = "UNKNOWN";
        switch (detected_format) {
            case H264_FORMAT_ANNEX_B:
                format_name = "Annex B (Start Codes) - CORRECT for MediaCodec";
                break;
            case H264_FORMAT_AVCC:
                format_name = "AVCC (Length-Prefixed) - NEEDS CONVERSION";
                break;
            default:
                format_name = "UNKNOWN - Cannot determine format";
                break;
        }

        spdlog::get("illixr")->info("H.264 Format Detected: {}", format_name);

        // Log first 32 bytes to help diagnose
        std::string hex_dump;
        for (size_t i = 0; i < std::min(encoded_data.size(), size_t(32)); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", encoded_data[i]);
            hex_dump += buf;
        }
        spdlog::get("illixr")->info("First 32 bytes: {}", hex_dump);

        format_detected = true;
    }

    // Convert AVCC to Annex B if needed
    if (detected_format == H264_FORMAT_AVCC) {
        encoded_data = convert_avcc_to_annexb(encoded_data.data(), encoded_data.size());
    } else if (detected_format == H264_FORMAT_UNKNOWN) {
        spdlog::get("illixr")->error("Cannot decode - unknown H.264 format!");
    }
}



// Find all NAL units in the data and log their types
void frame_decoder::analyze_h264_data(const uint8_t* data, size_t size, const char* label) {
    if (!data || size < 4) {
        spdlog::get("illixr")->warn("{}: Data too small ({} bytes)", label, size);
        return;
    }

    spdlog::get("illixr")->info("{}: Analyzing {} bytes", label, size);

    // Log first 16 bytes as hex
    std::string hex_dump;
    for (size_t i = 0; i < std::min(size, size_t(16)); i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", data[i]);
        hex_dump += buf;
    }
    spdlog::get("illixr")->info("{}: First bytes: {}", label, hex_dump);

    // Check for start codes and NAL units
    bool has_sps = false;
    bool has_pps = false;
    bool has_idr = false;
    int nal_count = 0;

    size_t i = 0;
    while (i < size - 4) {
        // Look for start code
        bool found_start = false;
        size_t start_code_len = 0;

        if (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x00 && data[i+3] == 0x01) {
            found_start = true;
            start_code_len = 4;
        } else if (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01) {
            found_start = true;
            start_code_len = 3;
        }

        if (found_start) {
            // Get NAL unit type
            uint8_t nal_header = data[i + start_code_len];
            uint8_t nal_type = get_nal_unit_type(nal_header);

            const char* nal_type_name = "UNKNOWN";
            switch (nal_type) {
                case NAL_UNIT_TYPE_CODED_SLICE_NON_IDR: nal_type_name = "NON-IDR Slice"; break;
                case NAL_UNIT_TYPE_CODED_SLICE_IDR:
                    nal_type_name = "IDR Slice (Keyframe)";
                    has_idr = true;
                    break;
                case NAL_UNIT_TYPE_SEI: nal_type_name = "SEI"; break;
                case NAL_UNIT_TYPE_SPS:
                    nal_type_name = "SPS (REQUIRED)";
                    has_sps = true;
                    break;
                case NAL_UNIT_TYPE_PPS:
                    nal_type_name = "PPS (REQUIRED)";
                    has_pps = true;
                    break;
                case NAL_UNIT_TYPE_AUD: nal_type_name = "AUD"; break;
                default: break;
            }

            spdlog::get("illixr")->info("{}: NAL #{} at offset {}: Type {} ({})",
                                        label, nal_count, i, nal_type, nal_type_name);
            nal_count++;

            i += start_code_len + 1;
        } else {
            i++;
        }
    }

    // Summary
    spdlog::get("illixr")->info("{}: Found {} NAL units", label, nal_count);
    spdlog::get("illixr")->info("{}: Has SPS: {}, Has PPS: {}, Has IDR: {}",
                                label, has_sps, has_pps, has_idr);

    if (!has_sps || !has_pps) {
        spdlog::get("illixr")->error("{}: MISSING REQUIRED HEADERS! SPS={} PPS={}",
                                     label, has_sps, has_pps);
        spdlog::get("illixr")->error("{}: MediaCodec WILL FAIL without SPS/PPS!", label);
    }
}

// Quick check if data has SPS and PPS
bool frame_decoder::has_sps_pps(const uint8_t* data, size_t size) {
    bool has_sps = false;
    bool has_pps = false;

    for (size_t i = 0; i < size - 4; i++) {
        if ((data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x00 && data[i+3] == 0x01) ||
            (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01)) {

            size_t offset = (data[i+2] == 0x01) ? 3 : 4;
            uint8_t nal_type = get_nal_unit_type(data[i + offset]);

            if (nal_type == NAL_UNIT_TYPE_SPS) has_sps = true;
            if (nal_type == NAL_UNIT_TYPE_PPS) has_pps = true;

            if (has_sps && has_pps) return true;
        }
    }

    return false;
}

// Extract SPS and PPS from data (if present)
bool frame_decoder::extract_sps_pps(const uint8_t* data, size_t size,
                     std::vector<uint8_t>& sps_out,
                     std::vector<uint8_t>& pps_out) {
    sps_out.clear();
    pps_out.clear();

    for (size_t i = 0; i < size - 4; i++) {
        size_t start_code_len = 0;

        if (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x00 && data[i+3] == 0x01) {
            start_code_len = 4;
        } else if (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01) {
            start_code_len = 3;
        } else {
            continue;
        }

        uint8_t nal_type = get_nal_unit_type(data[i + start_code_len]);

        // Find next start code to determine NAL unit length
        size_t next_start = size;
        for (size_t j = i + start_code_len + 1; j < size - 3; j++) {
            if ((data[j] == 0x00 && data[j+1] == 0x00 && data[j+2] == 0x01) ||
                (data[j] == 0x00 && data[j+1] == 0x00 && data[j+2] == 0x00 && data[j+3] == 0x01)) {
                next_start = j;
                break;
            }
        }

        size_t nal_size = next_start - i;

        if (nal_type == NAL_UNIT_TYPE_SPS) {
            sps_out.assign(data + i, data + i + nal_size);
            spdlog::get("illixr")->info("Extracted SPS: {} bytes", nal_size);
        } else if (nal_type == NAL_UNIT_TYPE_PPS) {
            pps_out.assign(data + i, data + i + nal_size);
            spdlog::get("illixr")->info("Extracted PPS: {} bytes", nal_size);
        }

        i = next_start - 1;
    }

    return !sps_out.empty() && !pps_out.empty();
}
// NV12 to RGBA conversion
std::vector<uint8_t> DecodedFrame::to_rgba() const {
    std::vector<uint8_t> rgba(width * height * 4);

    const uint8_t* yPlane = get_y_plane();
    const uint8_t* uvPlane = get_uv_plane();

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int yIdx = y * width + x;
            int uvIdx = (y / 2) * width + (x & ~1);

            int Y = yPlane[yIdx] - 16;
            int U = uvPlane[uvIdx] - 128;
            int V = uvPlane[uvIdx + 1] - 128;

            // YUV to RGB conversion
            int R = (298 * Y + 409 * V + 128) >> 8;
            int G = (298 * Y - 100 * U - 208 * V + 128) >> 8;
            int B = (298 * Y + 516 * U + 128) >> 8;

            // Clamp values
            R = std::max(0, std::min(255, R));
            G = std::max(0, std::min(255, G));
            B = std::max(0, std::min(255, B));

            int outIdx = (y * width + x) * 4;
            rgba[outIdx] = R;
            rgba[outIdx + 1] = G;
            rgba[outIdx + 2] = B;
            rgba[outIdx + 3] = 255; // Alpha
        }
    }

    return rgba;
}

// NV12 to RGB conversion (no alpha channel)
std::vector<uint8_t> DecodedFrame::to_rgb() const {
    std::vector<uint8_t> rgb(width * height * 3);

    const uint8_t* yPlane = get_y_plane();
    const uint8_t* uvPlane = get_uv_plane();

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int yIdx = y * width + x;
            int uvIdx = (y / 2) * width + (x & ~1);

            int Y = yPlane[yIdx] - 16;
            int U = uvPlane[uvIdx] - 128;
            int V = uvPlane[uvIdx + 1] - 128;

            // YUV to RGB conversion
            int R = (298 * Y + 409 * V + 128) >> 8;
            int G = (298 * Y - 100 * U - 208 * V + 128) >> 8;
            int B = (298 * Y + 516 * U + 128) >> 8;

            // Clamp values
            R = std::max(0, std::min(255, R));
            G = std::max(0, std::min(255, G));
            B = std::max(0, std::min(255, B));

            int outIdx = (y * width + x) * 3;
            rgb[outIdx] = R;
            rgb[outIdx + 1] = G;
            rgb[outIdx + 2] = B;
        }
    }

    return rgb;
}
void frame_decoder::decode_loop() {
    while (running_) {
        bool input_fed = false;
        //std::vector<uint8_t> encoded_data;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (!input_queue_.empty()) {
                std::vector<uint8_t> encoded_data = std::move(input_queue_.front());
                input_queue_.pop();
                lock.unlock(); // Release lock before codec operations

                //spdlog::get("illixr")->debug("adding frame {}", encoded_data.size());
                if (!encoded_data.empty()) {
                    static bool first_frame = true;
                    if (first_frame) {
                        //spdlog::get("illixr")->info("First frame received: {} bytes", encoded_data.size());
                        check_first_frame_compatibility(encoded_data);
                        check_and_convert_h264_format(encoded_data);
                        analyze_h264_data(encoded_data.data(), encoded_data.size(), "First Frame");
                        first_frame = false;
                    }

                    // Get input buffer (short timeout)
                    //spdlog::get("illixr")->debug("A");
                    // ANALYZE FIRST FEW FRAMES
                    static int frame_count = 0;
                    if (frame_count < 3) {  // Check first 3 frames
                        save_h264_for_analysis(encoded_data.data(), encoded_data.size(), frame_count);
                        analyze_h264_data(encoded_data.data(), encoded_data.size(),
                                          ("Frame " + std::to_string(frame_count)).c_str());
                        frame_count++;
                    }

                    // Check if this frame has SPS/PPS
                    if (!config_sent_) {
                        if (has_sps_pps(encoded_data.data(), encoded_data.size())) {
                            spdlog::get("illixr")->info("Found SPS/PPS in frame, extracting...");
                            extract_sps_pps(encoded_data.data(), encoded_data.size(),
                                            cached_sps_, cached_pps_);
                            config_sent_ = true;
                        } else {
                            spdlog::get("illixr")->warn("Frame missing SPS/PPS! Decoder may fail.");
                        }
                    }
                    ssize_t buffer_idx = AMediaCodec_dequeueInputBuffer(codec_, 1000);
                    if (buffer_idx >= 0) {
                        size_t buffer_size;
                        uint8_t* buffer = AMediaCodec_getInputBuffer(codec_, buffer_idx, &buffer_size);
                        //spdlog::get("illixr")->debug("B");
                        if (buffer && encoded_data.size() <= buffer_size) {
                            memcpy(buffer, encoded_data.data(), encoded_data.size());

                            media_status_t status = AMediaCodec_queueInputBuffer(
                                    codec_, buffer_idx, 0, encoded_data.size(), 0, 0);

                            if (status == AMEDIA_OK) {
                                //spdlog::get("illixr")->debug("Fed");
                                input_fed = true;
                            } else {
                                spdlog::get("illixr")->error("Failed to queue input buffer: {}",
                                                             static_cast<int>(status));
                            }
                        } else {
                            spdlog::get("illixr")->error("Input buffer too small or null");
                        }
                    }
                }
            }
            /*queue_cv_.wait(lock, [this] { return !input_queue_.empty() || !running_; });

            if (!running_ && input_queue_.empty())
                break;

            if (!input_queue_.empty()) {
                encoded_data = std::move(input_queue_.front());
                input_queue_.pop();
            }*/
        }

        //if (encoded_data.empty())
        //    continue;

        // Get input buffer
        /*ssize_t bufferIdx = AMediaCodec_dequeueInputBuffer(codec_, 10000);
        if (bufferIdx >= 0) {
            size_t bufferSize;
            uint8_t* buffer = AMediaCodec_getInputBuffer(codec_, bufferIdx, &bufferSize);

            if (buffer && encoded_data.size() <= bufferSize) {
                memcpy(buffer, encoded_data.data(), encoded_data.size());

                auto stat = AMediaCodec_queueInputBuffer(codec_, bufferIdx, 0,
                                             encoded_data.size(),
                                             0, 0);
                if (stat != AMEDIA_OK) {
                    spdlog::get("illixr")->debug("xyz");
                }
            } else {
                spdlog::get("illixr")->error("Input buffer too small or null");
            }
        }*/
        // Get output buffer
        AMediaCodecBufferInfo info;
        ssize_t output_idx = AMediaCodec_dequeueOutputBuffer(codec_, &info, 10000);

        if (output_idx >= 0) {
            spdlog::get("illixr")->debug("have idx");
            if (info.size > 0) {
                // Get raw buffer and invoke callback
                size_t bufferSize;
                uint8_t* buffer = AMediaCodec_getOutputBuffer(codec_, output_idx, &bufferSize);

                if (buffer) {
                    // Store in output queue for synchronized retrieval
                    {
                        std::lock_guard<std::mutex> lock(output_mutex_);
                        if (output_queue_.size() < max_output_queue_) {
                            DecodedFrame frame;
                            frame.data.assign(buffer, buffer + info.size);
                            frame.width = width_;
                            frame.height = height_;
                            frame.timestamp = info.presentationTimeUs;
                            output_queue_.push(std::move(frame));
                            output_cv_.notify_one();
                        }
                    }
                }
                AMediaCodec_releaseOutputBuffer(codec_, output_idx, false);
            }
        } else if (output_idx == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
            AMediaFormat* format = AMediaCodec_getOutputFormat(codec_);
            AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_WIDTH, &width_);
            AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_HEIGHT, &height_);
            int32_t colorFormat;
            AMediaFormat_getInt32(format, AMEDIAFORMAT_KEY_COLOR_FORMAT, &colorFormat);
            spdlog::get("illixr")->info("Output format changed: {}x{}, format {}", width_, height_, colorFormat);
            AMediaFormat_delete(format);
        } else if (output_idx == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
            // No output available yet - this is normal, especially at startup
            if (!input_fed) {
                //spdlog::get("illixr")->debug("try again, no input");
                // If we didn't feed input and no output, wait a bit for new input
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, std::chrono::milliseconds(10),
                                   [this] { return !input_queue_.empty() || !running_; });
            } else {
                //spdlog::get("illixr")->debug("try again");
            }
        } else {
            static int error_count = 0;
            if (error_count < 5) {  // Only log first 5 errors
                spdlog::get("illixr")->warn("dequeueOutputBuffer returned: {}",
                                            static_cast<int>(output_idx));
                error_count++;
            }
            spdlog::get("illixr")->warn("Unexpected dequeueOutputBuffer result: {}",
                                        static_cast<int>(output_idx));
        }
    }
}


frame_decoder::frame_decoder(int w, int h)
        : width_{w}
        , height_{h} {

    spdlog::get("illixr")->info("Creating H.264 decoder for {}x{}", w, h);

    // Create H.264 decoder
    codec_ = AMediaCodec_createCodecByName("OMX.qcom.video.decoder.avc");
    if (!codec_) {
        spdlog::get("illixr")->warn("Qualcomm hardware decoder not found, trying generic");
        codec_ = AMediaCodec_createDecoderByType("video/avc");
    }
    if (!codec_) {
        spdlog::get("illixr")->error("Failed to create decoder");
        throw std::runtime_error("Failed to create decoder");
    }

    // Configure decoder
    AMediaFormat* format = AMediaFormat_new();
    AMediaFormat_setString(format, AMEDIAFORMAT_KEY_MIME, "video/avc");
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_WIDTH, width_);
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_HEIGHT, height_);

    // Request NV12 output (color format 21)
    // This matches the input format from NVIDIA encoder
    // AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_COLOR_FORMAT, 21);
    AMediaFormat_setInt32(format, AMEDIAFORMAT_KEY_MAX_INPUT_SIZE, width_ * height_ * 2);
    media_status_t status;

    spdlog::get("illixr")->info("Configuring decoder: {}x{}, max_input_size: {}",
                                width_, height_, width_ * height_ * 2);
    // Configure for buffer output
    status = AMediaCodec_configure(codec_, format, nullptr, nullptr, 0);
    // Log configuration
    const char* config_str = AMediaFormat_toString(format);
    spdlog::get("illixr")->info("Decoder config: {}", config_str);

    AMediaFormat_delete(format);

    if (status != AMEDIA_OK) {
        spdlog::get("illixr")->error("Failed to configure decoder: {}", static_cast<int>(status));
        AMediaCodec_delete(codec_);
        throw std::runtime_error("Failed to configure decoder");
    }

    // Start decoder
    status = AMediaCodec_start(codec_);
    if (status != AMEDIA_OK) {
        spdlog::get("illixr")->error("Failed to start decoder: {}", static_cast<int>(status));
        AMediaCodec_delete(codec_);
        throw std::runtime_error("Failed to start decoder");
    }

    spdlog::get("illixr")->info("Decoder initialized successfully");

    // Start decode thread
    running_ = true;
    decode_thread_ = std::thread(&frame_decoder::decode_loop, this);
}

frame_decoder::~frame_decoder() {
    stop();

    if (codec_) {
        AMediaCodec_stop(codec_);
        AMediaCodec_delete(codec_);
    }
}

void frame_decoder::stop() {
    running_ = false;
    queue_cv_.notify_all();
    output_cv_.notify_all();

    if (decode_thread_.joinable()) {
        decode_thread_.join();
    }
}

// Queue encoded data for decoding
void frame_decoder::queue_frame(const std::vector<uint8_t>& encoded_data) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    input_queue_.push(encoded_data);
    queue_cv_.notify_one();
}

void frame_decoder::queue_frame(const uint8_t* data, size_t size) {
    std::vector<uint8_t> vec(data, data + size);
    queue_frame(vec);
}

void frame_decoder::flush() {
    if (codec_) {
        AMediaCodec_flush(codec_);
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!input_queue_.empty()) {
        input_queue_.pop();
    }

    std::lock_guard<std::mutex> outLock(output_mutex_);
    while (!output_queue_.empty()) {
        output_queue_.pop();
    }
}

bool frame_decoder::get_decoded_frame(DecodedFrame& frame, int timeout_mil) {
    std::unique_lock<std::mutex> lock(output_mutex_);

    if (timeout_mil < 0) {
        output_cv_.wait(lock, [this] { return !output_queue_.empty() || !running_; });
    } else {
        if (!output_cv_.wait_for(lock, std::chrono::milliseconds(timeout_mil),
                               [this] { return !output_queue_.empty() || !running_; })) {
            //spdlog::get("illixr")->debug("queue size: {}", output_queue_.size());
            return false;
        }
    }

    if (output_queue_.empty()) {
        return false;
    }

    frame = std::move(output_queue_.front());
    output_queue_.pop();
    return true;
}

bool frame_decoder::has_decoded_frame() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return !output_queue_.empty();
}

size_t frame_decoder::get_decoded_frame_count() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return output_queue_.size();
}
