#pragma once

#include "illixr/data_format/imu.hpp"
#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"

#include <gtsam/navigation/ImuBias.h>

namespace ILLIXR {
class gtsam_integrator : public plugin {
public:
    [[maybe_unused]] gtsam_integrator(const std::string& name_, phonebook* pb_);

    void callback(switchboard::ptr<const data_format::imu_type> datum);

private:
    const std::shared_ptr<switchboard>   switchboard_;
    const std::shared_ptr<relative_clock> clock_;

    // IMU Data, Sequence Flag, and State Vars Needed
    switchboard::reader<data_format::imu_integrator_input> imu_integrator_input_;

    // Write IMU Biases for PP
    switchboard::writer<data_format::imu_raw_type> imu_raw_;

    std::vector<data_format::imu_type> imu_vec_;

    [[maybe_unused]] time_point last_cam_time_;
    duration                    last_imu_offset_;

    /**
     * @brief Wrapper object protecting the lifetime of IMU integration inputs and biases
     */
    class PimObject {
    public:
        using imu_int_t = ILLIXR::data_format::imu_integrator_input;
        using imu_t     = data_format::imu_type;
        using bias_t    = ImuBias;
        using nav_t     = gtsam::NavState;
        using pim_t     = gtsam::PreintegratedCombinedMeasurements;
        using pim_ptr_t = gtsam::PreintegrationType*;

        PimObject(const imu_int_t& imu_int_input);

        ~PimObject();

        void resetIntegrationAndSetBias(const imu_int_t& imu_int_input) noexcept;

        void integrateMeasurement(const imu_t& imu_input, const imu_t& imu_input_next) noexcept;

        bias_t biasHat() const noexcept;
        nav_t predict() const noexcept;
    private:
        bias_t    imu_bias_;
        nav_t     navstate_lkf_;
        pim_ptr_t pim_;
    };

    std::unique_ptr<PimObject> _pim_obj;

    // Remove IMU values older than 'IMU_TTL' from the imu buffer
    void clean_imu_vec(time_point timestamp);

    // Timestamp we are propagating the biases to (new IMU reading time)
    void propagate_imu_values(time_point real_time);

    // Select IMU readings based on timestamp similar to how OpenVINS selects IMU values to propagate
    std::vector<data_format::imu_type> select_imu_readings(const std::vector<data_format::imu_type>& imu_data, const time_point time_begin,
                                                           const time_point time_end);

    // For when an integration time ever falls inbetween two imu measurements (modeled after OpenVINS)
    static data_format::imu_type interpolate_imu(const data_format::imu_type& imu_1, const data_format::imu_type& imu_2, time_point timestamp);
};

}