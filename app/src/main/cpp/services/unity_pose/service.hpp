#pragma once
#define UNITY_POSE_RECEIVER

#include "illixr/data_format/pose_prediction.hpp"
#include "illixr/data_format/misc.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/plugin.hpp"

namespace ILLIXR {

class unity_pose_impl : public data_format::pose_prediction {
public:
    explicit unity_pose_impl(const phonebook* pb);
    data_format::fast_pose_type get_fast_pose() const override;
    data_format::pose_type get_true_pose() const override;
    bool fast_pose_reliable() const override;
    bool true_pose_reliable() const override;
    Eigen::Quaternionf get_offset() override;
    data_format::pose_type correct_pose(const data_format::pose_type& pose) const override;
    void set_offset(const Eigen::Quaternionf& raw_o_times_offset) override;
    Eigen::Quaternionf apply_offset(const Eigen::Quaternionf& orientation) const;
    data_format::fast_pose_type get_fast_pose(time_point time) const override;
    static void set_current_pose(data_format::unity_pose& pose);
private:
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<const relative_clock> clock_;
    mutable Eigen::Quaternionf                  offset_{Eigen::Quaternionf::Identity()};
    mutable std::shared_mutex                   offset_mutex_;

    static data_format::pose_type current_pose_;
    switchboard::reader<switchboard::event_wrapper<time_point>> vsync_estimate_;
};

class unity_pose_plugin : public plugin {
public:
    [[maybe_unused]] unity_pose_plugin(const std::string& name, phonebook* pb);
    std::function<void(data_format::unity_pose&)> get_pose_function() override;
    bool gets_unity_pose() const override;
private:
    std::shared_ptr<unity_pose_impl> pose_impl_;
};

#undef UNITY_POSE_RECEIVER
}