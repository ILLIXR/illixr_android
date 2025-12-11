#pragma once

#include "illixr/data_format/frame.hpp"
#include "illixr/network/net_config.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/quest3_params.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

#include "stereo_surface_decoder.hpp"

#if __has_include("rendered_frame.pb.h")
#include "rendered_frame.pb.h"
#else
#include "../proto/rendered_frame_stub.hpp"
#endif


namespace ILLIXR {

class android_media_decoder : public threadloop {
public:
    [[maybe_unused]] android_media_decoder(const std::string& name, phonebook* pb);

    ~android_media_decoder();
protected:
    void _p_one_iteration() override;
    void _p_thread_setup() override;
private:
    void decompress_frame(const rendered_frame_proto::CompressedFrame& frame);

    void send_frame();

    const std::shared_ptr<switchboard>                                    switchboard_;
    const std::shared_ptr<relative_clock>                                 clock_;

    switchboard::writer<data_format::dual_frames>                         frame_writer_;
    switchboard::buffered_reader<switchboard::event_wrapper<std::string>> compressed_frame_reader_;

    android_app* app_;
    std::string buffer_str_;

    std::unique_ptr<stereo_surface_decoder> decoder_;
    bool decoder_initialized_{false};
    const std::string delimiter_ = "END!";
};

}
