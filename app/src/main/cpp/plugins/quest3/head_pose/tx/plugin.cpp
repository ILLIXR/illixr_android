#include "plugin.hpp"

using namespace ILLIXR;
using namespace ILLIXR::data_format;

[[maybe_unused]] quest_pose_tx::quest_pose_tx(const std::string& name, phonebook* pb)
        : plugin{name, pb}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , clock_{phonebook_->lookup_impl<relative_clock>()}
        , writer_{switchboard_->get_network_writer<switchboard::event_wrapper<std::string>>(
                "tx_xr_pose",
                network::topic_config{network::topic_config::priority_type::MEDIUM,
                                      false,
                                      false,
                                      network::topic_config::packetization_type::DEFAULT,
                                      {},
                                      network::topic_config::SerializationMethod::PROTOBUF})} {
    switchboard_->schedule<pose_type>(id_, "xr_pose", [&](const switchboard::ptr<const pose_type>& datum, size_t) {
        callback(datum);
    });
}

void quest_pose_tx::callback(const switchboard::ptr<const data_format::pose_type>& datum) {
    std::string                     message;
    //spdlog::get("illixr")->debug("received pose");
    auto* pose = new unity_pose_proto::Pose();
    pose->set_timestamp(datum->sensor_time.time_since_epoch().count());

    auto* position = new unity_pose_proto::Position();
    position->set_x(datum->position.x());
    position->set_y(datum->position.y());
    position->set_z(datum->position.z());
    pose->set_allocated_pos(position);

    auto* quat = new unity_pose_proto::Quaternion();
    quat->set_w(datum->orientation.w());
    quat->set_x(datum->orientation.x());
    quat->set_y(datum->orientation.y());
    quat->set_z(datum->orientation.z());
    pose->set_allocated_quat(quat);

    //spdlog::get("illixr")->debug("pose is: {}, {}, {}: {}, {}, {}, {}", datum->position.x(), datum->position.y(),
    //                             datum->position.z(), datum->orientation.w(), datum->orientation.x(),
    //                             datum->orientation.y(), datum->orientation.z());
    message = pose->SerializeAsString();
    //message += delimiter_;
    //spdlog::get("illixr")->info("Device Sending Pose size: {}", message.size());
    writer_.put(std::make_shared<switchboard::event_wrapper<std::string>>(message + delimiter_));
    delete pose;
}

PLUGIN_MAIN(quest_pose_tx)

