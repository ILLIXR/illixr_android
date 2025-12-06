#include "plugin.hpp"

using namespace ILLIXR;
/*
void toRGBA8(DecodedFrame& frame, std::vector<uint8_t>& out_image) {
    // Assume NV21 format (YUV420sp)
    out_image.resize(frame.width * frame.height * 4);

    const uint8_t* y = frame.data.data();
    const uint8_t* vu = frame.data.data() + frame.width * frame.height;

    for (int row = 0; row < frame.height; row++) {
        for (int col = 0; col < frame.width; col++) {
            int yIndex = row * frame.width + col;
            int uvIndex = (row / 2) * frame.width + (col & ~1);

            int Y = y[yIndex];
            int V = vu[uvIndex];
            int U = vu[uvIndex + 1];

            // YUV to RGB conversion
            int C = Y - 16;
            int D = U - 128;
            int E = V - 128;

            int R = (298 * C + 409 * E + 128) >> 8;
            int G = (298 * C - 100 * D - 208 * E + 128) >> 8;
            int B = (298 * C + 516 * D + 128) >> 8;

            // Clamp values
            R = R < 0 ? 0 : (R > 255 ? 255 : R);
            G = G < 0 ? 0 : (G > 255 ? 255 : G);
            B = B < 0 ? 0 : (B > 255 ? 255 : B);

            int rgbaIndex = yIndex * 4;
            out_image[rgbaIndex + 0] = R;
            out_image[rgbaIndex + 1] = G;
            out_image[rgbaIndex + 2] = B;
            out_image[rgbaIndex + 3] = 255; // Alpha
        }
    }
}*/

[[maybe_unused]] android_media_decoder::android_media_decoder(const std::string& name, phonebook* pb)
        : threadloop{name, pb}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , clock_{pb->lookup_impl<relative_clock>()}
        , frame_writer_{switchboard_->get_writer<data_format::dual_frames>("unity_rendered_frame")}
        , compressed_frame_reader_{switchboard_->get_buffered_reader<switchboard::event_wrapper<std::string>>("compressed_frame")} {
    left_decoder_ = std::make_unique<frame_decoder>(WIDTH, HEIGHT);
    right_decoder_ = std::make_unique<frame_decoder>(WIDTH, HEIGHT);
}

android_media_decoder::~android_media_decoder() {

}

void android_media_decoder::_p_one_iteration() {
    if (compressed_frame_reader_.size() > 0) {
        auto buffer_ptr = compressed_frame_reader_.dequeue();
        buffer_str_ = **buffer_ptr;
        std::string::size_type end_position = buffer_str_.find(delimiter_);
        rendered_frame_proto::CompressedFrame frame;
        if (frame.ParseFromString(buffer_str_.substr(0, end_position))) {
            spdlog::get("illixr")->debug("Got frame");
            decompress_frame(frame);
        } else {
            spdlog::get("illixr")->error("errrr");
        }
    }
}

void android_media_decoder::decompress_frame(const rendered_frame_proto::CompressedFrame& frame) {
    left_decoder_->queue_frame(reinterpret_cast<const uint8_t*>(frame.left_eye().data()),
                               frame.left_eye_size());
    right_decoder_->queue_frame(reinterpret_cast<const uint8_t*>(frame.right_eye().data()),
                                frame.right_eye_size());
    bool got_left = left_decoder_->get_decoded_frame(left_data_, 300);
    bool got_right = right_decoder_->get_decoded_frame(right_data_, 300);
    if (got_left && got_right) {
        send_frame();
        spdlog::get("illixr")->debug("++++++  ----------    frames");
    } else {
        spdlog::get("illixr")->error("Decompression failed");
    }
}

void android_media_decoder::send_frame() {
    //std::vector<uint8_t> left_buffer, right_buffer;
    //left_buffer = left_data_.to_rgba();
    //right_buffer = right_data_.to_rgba();
    //toRGBA8(left_data_, left_buffer);
    //toRGBA8(right_data_, right_buffer);
    frame_writer_.put(frame_writer_.allocate<data_format::dual_frames>(data_format::dual_frames{clock_->now(), left_data_.data, right_data_.data, WIDTH, HEIGHT}));

}

PLUGIN_MAIN(android_media_decoder)
