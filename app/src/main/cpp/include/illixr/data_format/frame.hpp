#pragma once

#ifdef Success
#undef Success // For 'Success' conflict
#endif

#include "illixr/data_format/pose.hpp"
#include "illixr/switchboard.hpp"

#include <array>
#include <boost/serialization/export.hpp>
#include <GLES/gl.h>

#ifdef ILLIXR_LIBAV
extern "C" {
    #include "libavcodec_illixr/avcodec.h"
    #include "libavformat_illixr/avformat.h"
    #include "libavutil_illixr/hwcontext.h"
    #include "libavutil_illixr/opt.h"
    #include "libavutil_illixr/pixdesc.h"
}
#endif

namespace ILLIXR::data_format {
// Using arrays as a swapchain
// Array of left eyes, array of right eyes
// This more closely matches the format used by Monado
struct [[maybe_unused]] rendered_frame : public switchboard::event {
    std::array<GLuint, 2> swapchain_indices{}; // Index of image rendered for left and right swapchain
    std::array<GLuint, 2> swap_indices{};    // Which element of the swapchain
    fast_pose_type        render_pose;     // The pose used when rendering this frame.
    time_point            sample_time{};
    time_point            render_time{};

    rendered_frame() = default;

    rendered_frame(std::array<GLuint, 2>&& swapchain_indices_, std::array<GLuint, 2>&& swap_indices_,
                   fast_pose_type render_pose_, time_point sample_time_, time_point render_time_)
            : swapchain_indices{swapchain_indices_}
            , swap_indices{swap_indices_}
            , render_pose(std::move(render_pose_))
            , sample_time(sample_time_)
            , render_time(render_time_) { }
};

struct compressed_frame : public switchboard::event {
    bool  nalu_only;
    char* left_color_nalu  = nullptr;
    char* right_color_nalu = nullptr;
    char* left_depth_nalu  = nullptr;
    char* right_depth_nalu = nullptr;
    int   left_color_nalu_size;
    int   right_color_nalu_size;
    int   left_depth_nalu_size;
    int   right_depth_nalu_size;

    bool use_depth;
#ifdef ILLIXR_LIBAV
    AVPacket* left_color;
    AVPacket* right_color;

    AVPacket* left_depth;
    AVPacket* right_depth;
#endif

    fast_pose_type pose;
    uint64_t       sent_time;
    long           magic = 0;

    friend class boost::serialization::access;

#ifdef ILLIXR_LIBAV
    template<class Archive>
    static void save_packet(Archive& ar, AVPacket* pkt) {
        ar << pkt->size;
        ar << boost::serialization::make_array(pkt->data, pkt->size);
        ar << pkt->pts;
        ar << pkt->dts;
        ar << pkt->stream_index;
        ar << pkt->flags;
        ar << pkt->duration;
        ar << pkt->pos;
        ar << pkt->time_base.num;
        ar << pkt->time_base.den;
        ar << pkt->side_data_elems;
        for (int i = 0; i < pkt->side_data_elems; i++) {
            ar << pkt->side_data[i].type;
            ar << pkt->side_data[i].size;
            ar << boost::serialization::make_array(pkt->side_data[i].data, pkt->side_data[i].size);
        }
    }

    template<class Archive>
    static void load_packet(Archive& ar, AVPacket* pkt) {
        ar >> pkt->size;
        pkt->buf  = av_buffer_alloc(pkt->size);
        pkt->data = pkt->buf->data;
        ar >> boost::serialization::make_array(pkt->data, pkt->size);
        ar >> pkt->pts;
        ar >> pkt->dts;
        ar >> pkt->stream_index;
        ar >> pkt->flags;
        ar >> pkt->duration;
        ar >> pkt->pos;
        ar >> pkt->time_base.num;
        ar >> pkt->time_base.den;
        ar >> pkt->side_data_elems;
        pkt->side_data = (AVPacketSideData*) malloc(sizeof(AVPacketSideData) * pkt->side_data_elems);
        for (int i = 0; i < pkt->side_data_elems; i++) {
            ar >> pkt->side_data[i].type;
            ar >> pkt->side_data[i].size;
            pkt->side_data[i].data = (uint8_t*) malloc(pkt->side_data[i].size);
            ar >> boost::serialization::make_array(pkt->side_data[i].data, pkt->side_data[i].size);
        }
    }
#endif

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const {
        (void) version;
        ar << boost::serialization::base_object<switchboard::event>(*this);
        ar << nalu_only;
        ar << use_depth;
#ifdef ILLIXR_LIBAV
        if (nalu_only) {
            ar << left_color->size;
            ar << right_color->size;
            ar << boost::serialization::make_array(left_color->data, left_color->size);
            ar << boost::serialization::make_array(right_color->data, right_color->size);
            if (use_depth) {
                ar << left_depth->size;
                ar << right_depth->size;
                ar << boost::serialization::make_array(left_depth->data, left_depth->size);
                ar << boost::serialization::make_array(right_depth->data, right_depth->size);
            }
        } else {
            save_packet(ar, left_color);
            save_packet(ar, right_color);
            if (use_depth) {
                save_packet(ar, left_depth);
                save_packet(ar, right_depth);
            }
        }
#else
        assert(false && "Not compiled with libav");
#endif

        ar << pose;
        ar << sent_time;
        ar << magic;
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version) {
        (void) version;
        ar >> boost::serialization::base_object<switchboard::event>(*this);
        ar >> nalu_only;
        ar >> use_depth;
        if (nalu_only) {
            ar >> left_color_nalu_size;
            ar >> right_color_nalu_size;
            left_color_nalu  = (char*) malloc(left_color_nalu_size);
            right_color_nalu = (char*) malloc(right_color_nalu_size);
            ar >> boost::serialization::make_array(left_color_nalu, left_color_nalu_size);
            ar >> boost::serialization::make_array(right_color_nalu, right_color_nalu_size);
            if (use_depth) {
                ar >> left_depth_nalu_size;
                ar >> right_depth_nalu_size;
                left_depth_nalu  = (char*) malloc(left_depth_nalu_size);
                right_depth_nalu = (char*) malloc(right_depth_nalu_size);
                ar >> boost::serialization::make_array(left_depth_nalu, left_depth_nalu_size);
                ar >> boost::serialization::make_array(right_depth_nalu, right_depth_nalu_size);
            }
        } else {
#ifdef ILLIXR_LIBAV

            left_color = av_packet_alloc();
            load_packet(ar, left_color);
            right_color = av_packet_alloc();
            load_packet(ar, right_color);
            if (use_depth) {
                left_depth = av_packet_alloc();
                load_packet(ar, left_depth);
                right_depth = av_packet_alloc();
                load_packet(ar, right_depth);
            }
#else
            assert(false && "Not compiled with libav");
#endif
        }

        ar >> pose;
        ar >> sent_time;
        ar >> magic;
        if (magic != 0xdeadbeef) {
            throw std::runtime_error("Magic number mismatch");
        }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    compressed_frame() = default;

#ifdef ILLIXR_LIBAV
    compressed_frame(AVPacket* left_color, AVPacket* right_color, const fast_pose_type& pose, uint64_t sent_time,
                     bool nalu_only = false)
        : nalu_only(nalu_only)
        , use_depth(false)
        , left_color(left_color)
        , right_color(right_color)
        , left_depth(nullptr)
        , right_depth(nullptr)
        , pose(pose)
        , sent_time(sent_time)
        , magic(0xdeadbeef) { }

    compressed_frame(AVPacket* left_color, AVPacket* right_color, AVPacket* left_depth, AVPacket* right_depth,
                     const fast_pose_type& pose, uint64_t sent_time, bool nalu_only = false)
        : nalu_only(nalu_only)
        , use_depth(true)
        , left_color(left_color)
        , right_color(right_color)
        , left_depth(left_depth)
        , right_depth(right_depth)
        , pose(pose)
        , sent_time(sent_time)
        , magic(0xdeadbeef) { }
#endif
    ~compressed_frame() {
        if (nalu_only && left_color_nalu != nullptr && right_color_nalu != nullptr) {
            free(left_color_nalu);
            free(right_color_nalu);
            if (use_depth) {
                free(left_depth_nalu);
                free(right_depth_nalu);
            }
        }
    }
};

/// Decoded video frame data format
enum class frame_format : uint8_t {
    nv12,           ///< NV12 (Y plane + interleaved UV plane)
    rgba8,          ///< RGBA 8-bit per channel
    external_oes    ///< GL_TEXTURE_EXTERNAL_OES handle (for zero-copy path)
};

/// Single eye frame data
/// Can contain either raw pixel data or a GL texture handle
struct eye_frame {
    /// Raw pixel data (used when format is nv12 or rgba8)
    std::vector<uint8_t> data{};

    /// GL texture ID (used when format is external_oes)
    /// This texture is owned by the decoder and valid until the next frame
    GLuint texture_id{0};

    /// Texture transform matrix from SurfaceTexture (4x4, column-major)
    /// Only valid when format is external_oes
    std::array<float, 16> texture_transform{
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
    };

    eye_frame() = default;

    /// Construct with raw data
    explicit eye_frame(std::vector<uint8_t> raw_data)
            : data{std::move(raw_data)}
            , texture_id{0} {}

    /// Construct with texture handle and transform
    eye_frame(GLuint tex_id, const float* transform)
            : texture_id{tex_id} {
        if (transform) {
            std::copy(transform, transform + 16, texture_transform.begin());
        }
    }

    /// Check if this frame has valid data
    [[nodiscard]] bool has_data() const {
        return !data.empty() || texture_id != 0;
    }

    /// Get Y plane pointer for NV12 format
    [[nodiscard]] const uint8_t* get_y_plane() const {
        return data.data();
    }

    /// Get UV plane pointer for NV12 format
    [[nodiscard]] const uint8_t* get_uv_plane(int width, int height) const {
        return data.data() + (width * height);
    }
};

/// Dual-eye video frame for stereo VR rendering.
/// Published by the decoder plugin, consumed by the renderer.
struct [[maybe_unused]] dual_frames : public switchboard::event {
    eye_frame left_eye{};
    eye_frame right_eye{};

    int width{0};
    int height{0};

    /// Format of the frame data
    frame_format format{frame_format::nv12};

    /// Presentation timestamp
    time_point render_time{};

    /// Frame sequence number (for debugging/sync)
    uint64_t frame_number{0};

    dual_frames() = default;

    /// Construct with raw NV12 data
    dual_frames(time_point tp, std::vector<uint8_t> left, std::vector<uint8_t> right,
                int w, int h, uint64_t frame_num = 0)
            : left_eye{std::move(left)}
            , right_eye{std::move(right)}
            , width{w}
            , height{h}
            , format{frame_format::nv12}
            , render_time{tp}
            , frame_number{frame_num} {}

    /// Construct with external OES texture handles
    dual_frames(time_point tp, GLuint left_tex, const float* left_transform,
                GLuint right_tex, const float* right_transform,
                int w, int h, uint64_t frame_num = 0)
            : left_eye{left_tex, left_transform}
            , right_eye{right_tex, right_transform}
            , width{w}
            , height{h}
            , format{frame_format::external_oes}
            , render_time{tp}
            , frame_number{frame_num} {}

    /// Check if both eyes have valid data
    [[nodiscard]] bool is_valid() const {
        return left_eye.has_data() && right_eye.has_data() && width > 0 && height > 0;
    }
};

/*
struct [[maybe_unused]] dual_frames : public switchboard::event {
    std::vector<uint8_t> left_eye{};
    std::vector<uint8_t> right_eye{};
    int width;
    int height;
    time_point render_time{};

    dual_frames() = default;

    dual_frames(time_point tp, std::vector<uint8_t>& l, std::vector<uint8_t> r,
                int w, int h)
                : left_eye{l}
                , right_eye{r}
                , width{w}
                , height{h}
                , render_time{tp} {}

};*/
}