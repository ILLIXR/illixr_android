#pragma once

#include "illixr/data_format/frame.hpp"
#include "illixr/network/net_config.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/threadloop.hpp"
#include "illixr/switchboard.hpp"

#include "frame_decoder.hpp"

#if __has_include("rendered_frame.pb.h")
#include "rendered_frame.pb.h"
#else
#include "../proto/rendered_frame_stub.hpp"
#endif

#define WIDTH 2064/2
#define HEIGHT 2208/2

namespace ILLIXR {

class android_media_decoder : public threadloop {
public:
    [[maybe_unused]] android_media_decoder(const std::string& name, phonebook* pb);

    ~android_media_decoder();
protected:
    void _p_one_iteration() override;
private:
    void decompress_frame(const rendered_frame_proto::CompressedFrame& frame);

    void send_frame();

    const std::shared_ptr<switchboard>                                    switchboard_;
    const std::shared_ptr<relative_clock>                                 clock_;

    switchboard::writer<data_format::dual_frames>                         frame_writer_;
    switchboard::buffered_reader<switchboard::event_wrapper<std::string>> compressed_frame_reader_;
    std::unique_ptr<frame_decoder> left_decoder_ = nullptr;
    std::unique_ptr<frame_decoder> right_decoder_ = nullptr;
    std::string buffer_str_;

    DecodedFrame left_data_;
    DecodedFrame right_data_;
    const std::string delimiter_ = "END!";
};

}
