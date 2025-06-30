#pragma once

#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format/imu.hpp"

namespace ILLIXR {

class rk4_integrator : public plugin {
public:
    [[maybe_unused]] rk4_integrator(const std::string& name_, phonebook* pb_);

    void callback(switchboard::ptr<const data_format::imu_type> datum);

private:
    const std::shared_ptr<switchboard> switchboard_;
//    const std::shared_ptr<log_service> sl;

    // IMU Data, Sequence Flag, and State Vars Needed
    switchboard::reader<data_format::imu_integrator_input> imu_integrator_input_;

    // IMU Biases
    switchboard::writer<data_format::imu_raw_type> imu_raw_;
    std::vector<data_format::imu_type>             imu_vec_;
    duration                          last_imu_offset_;
    bool                              has_last_offset_ = false;

    [[maybe_unused]] int    counter_       = 0;
    [[maybe_unused]] int    cam_count_     = 0;
    [[maybe_unused]] int    total_imu_     = 0;
    [[maybe_unused]] double last_cam_time_ = 0;

    // Clean IMU values older than IMU_SAMPLE_LIFETIME seconds
    void clean_imu_vec(time_point timestamp);

    // Timestamp we are propagating the biases to (new IMU reading time)
    void propagate_imu_values(time_point real_time);

    // Select IMU readings based on timestamp similar to how OpenVINS selects IMU values to propagate
    std::vector<data_format::imu_type> select_imu_readings(const std::vector<data_format::imu_type>& imu_data, time_point time_begin,
                                                           time_point time_end);

    // For when an integration time ever falls inbetween two imu measurements (modeled after OpenVINS)
    static data_format::imu_type interpolate_imu(const data_format::imu_type& imu_1, const data_format::imu_type& imu_2, time_point timestamp);

}