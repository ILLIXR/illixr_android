#include "service.hpp"

using namespace ILLIXR;
using namespace ILLIXR::data_format;

pose_type unity_pose_impl::current_pose_{};
unity_pose_impl::unity_pose_impl(const ILLIXR::phonebook* pb)
        : switchboard_{pb->lookup_impl<switchboard>()}
        , clock_{pb->lookup_impl<relative_clock>()}
        , vsync_estimate_{switchboard_->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")} {
}

fast_pose_type unity_pose_impl::get_fast_pose() const {
    const switchboard::ptr<const switchboard::event_wrapper<time_point>> estimated_vsync = vsync_estimate_.get_ro_nullable();
    if (estimated_vsync == nullptr) {
        return get_fast_pose(clock_->now());
    } else {
        return get_fast_pose(estimated_vsync.get()->operator time_point());
    }
}

pose_type unity_pose_impl::get_true_pose() const {
    throw std::logic_error{"Not Implemented"};
}

bool unity_pose_impl::fast_pose_reliable() const {
    return true;
}

bool unity_pose_impl::true_pose_reliable() const {
    return false;
}

Eigen::Quaternionf unity_pose_impl::get_offset() {
    return offset_;
}

pose_type unity_pose_impl::correct_pose(const pose_type& pose) const {
    return pose;
}

void unity_pose_impl::set_offset(const Eigen::Quaternionf& raw_o_times_offset) {
    std::unique_lock   lock{offset_mutex_};
    Eigen::Quaternionf raw_o = raw_o_times_offset * offset_.inverse();
    offset_                  = raw_o.inverse();
}

Eigen::Quaternionf unity_pose_impl::apply_offset(const Eigen::Quaternionf& orientation) const {
    std::shared_lock lock{offset_mutex_};
    return orientation * offset_;
}

fast_pose_type unity_pose_impl::get_fast_pose(time_point time) const {
    return fast_pose_type{current_pose_, current_pose_.sensor_time, time};
}

void unity_pose_impl::set_current_pose(data_format::unity_pose& pose) {
    current_pose_ = pose_type{pose};
}


unity_pose_plugin::unity_pose_plugin(const std::string& name, phonebook* pb)
        : plugin{name, pb} {
    pose_impl_ = std::make_shared<unity_pose_impl>(pb);
    pb->register_impl<pose_prediction>(
            std::static_pointer_cast<pose_prediction>(pose_impl_));
}

std::function<void(unity_pose&)> unity_pose_plugin::get_pose_function() {
    return &pose_impl_->set_current_pose;
}

bool unity_pose_plugin::gets_unity_pose() const {
    return true;
}

PLUGIN_MAIN(unity_pose_plugin)
