#pragma once

#include "illixr/data_format/opencv_data_types.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

#include "data_loading.hpp"

namespace ILLIXR {
class offline_cam : public threadloop {
public:
    [[maybe_unused]] offline_cam(const std::string& name_, phonebook* pb_);

    skip_option _p_should_skip() override;

    void _p_one_iteration() override;

private:
    const std::shared_ptr<switchboard> switchboard_;
    switchboard::writer<data_format::cam_type> cam_publisher_;
    const std::map<ullong, sensor_types> sensor_data_;
    ullong dataset_first_time_;
    ullong last_ts_;
    std::shared_ptr<RelativeClock> clock_;
    std::map<ullong, sensor_types>::const_iterator next_row_;
};

}