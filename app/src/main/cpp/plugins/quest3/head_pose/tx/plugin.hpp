#pragma once

#include "illixr/data_format/pose.hpp"
#include "illixr/relative_clock.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/plugin.hpp"

#if __has_include("quest_pose.pb.h")
#include "quest_pose.pb.h"
#else
#include "../proto/quest_pose_stub.hpp"
#endif

namespace ILLIXR{

class quest_pose_tx : public plugin {
public:
    [[maybe_unused]] quest_pose_tx(const std::string& name, phonebook* pb);
    void callback(const switchboard::ptr<const data_format::pose_type>& datum);

private:
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<relative_clock> clock_;
    switchboard::network_writer<switchboard::event_wrapper<std::string> > writer_;
    const std::string delimiter_ = "EEND!";

};
}