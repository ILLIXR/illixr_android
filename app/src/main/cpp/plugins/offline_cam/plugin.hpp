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
    const std::map<ullong, sensor_types> sensor_data_;
    std::map<ullong, sensor_types>::const_iterator sensor_data_it_;
    const std::shared_ptr<switchboard> switchboard_;
    std::shared_ptr<relative_clock> clock_;
    switchboard::writer<data_format::binocular_cam_type> cam_publisher_;
    ullong dataset_first_time_;
    ullong dataset_now_;

};

}