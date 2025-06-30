#include "plugin.hpp"

using namespace ILLIXR;

[[maybe_unused]] offline_imu::offline_imu(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}, sensor_data_{load_data()}, sensor_data_it_{sensor_data_.cbegin()},
          switchboard_{pb_->lookup_impl<switchboard>()}, clock_{pb_->lookup_impl<relative_clock>()},
          imu_{switchboard_->get_writer<data_format::imu_type>("imu")},
          dataset_first_time_{sensor_data_it_->first}, dataset_now_{0},
          imu_cam_log_{record_logger_} {}


threadloop::skip_option offline_imu::_p_should_skip() {
    if (sensor_data_it_ != sensor_data_.end()) {
        assert(dataset_now_ < sensor_data_it_->first);
        dataset_now_ = sensor_data_it_->first;
        // Sleep for the difference between the current IMU vs 1st IMU and current UNIX time vs UNIX time the component was
        // init
        std::this_thread::sleep_for(
                time_point{std::chrono::nanoseconds{dataset_now_ - dataset_first_time_}} -
                clock_->now());

        return skip_option::run;

    } else {
        return skip_option::stop;
    }
}

void offline_imu::_p_one_iteration() {
    //auto start = std::chrono::high_resolution_clock::now();
    RAC_ERRNO_MSG("offline_imu at start of _p_one_iteration");
    assert(sensor_data_it_ != sensor_data_.end());
#ifndef NDEBUG
    std::chrono::time_point<std::chrono::nanoseconds> tp_dataset_now{
            std::chrono::nanoseconds{dataset_now_}};
    std::cerr << " IMU time: " << tp_dataset_now.time_since_epoch().count() << std::endl;
#endif

    const sensor_types& sensor_datum = sensor_data_it_->second;

    imu_.put(imu_.allocate<data_format::imu_type>(
            data_format::imu_type{
                    time_point{std::chrono::nanoseconds(dataset_now_ - dataset_first_time_)},
                    (sensor_datum.imu0.angular_v),
                    (sensor_datum.imu0.linear_a)}));
    ++sensor_data_it_;
}


PLUGIN_MAIN(offline_imu)
