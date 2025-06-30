#pragma once

#include "illixr/data_format/imu.hpp"
#include "illixr/threadloop.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/phonebook.hpp"

#include "data_loading.hpp"

namespace ILLIXR {
class offline_imu : public ILLIXR::threadloop {
public:
    [[maybe_unused]] offline_imu(const std::string& name_, phonebook* pb_);

protected:
    skip_option _p_should_skip() override;

    void _p_one_iteration() override;

private:
    const std::map<ullong, sensor_types> sensor_data_;
    std::map<ullong, sensor_types>::const_iterator sensor_data_it_;
    const std::shared_ptr<switchboard> switchboard_;
    std::shared_ptr<const relative_clock> clock_;
    switchboard::writer<data_format::imu_type> imu_;

    // Timestamp of the first IMU value from the dataset
    ullong dataset_first_time_;
    // Current IMU timestamp
    ullong dataset_now_;

    record_coalescer imu_cam_log_;
};

}