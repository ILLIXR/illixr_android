#include "plugin.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <shared_mutex>

using namespace ILLIXR;

[[maybe_unused]] offline_cam::offline_cam(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}, sensor_data_{load_data()}, sensor_data_it_{sensor_data_.cbegin()},
          switchboard_{pb_->lookup_impl<switchboard>()}, clock_{pb_->lookup_impl<relative_clock>()},
          cam_publisher_{switchboard_->get_writer<data_format::binocular_cam_type>("cam")},
          dataset_first_time_{sensor_data_.cbegin()->first}, dataset_now_{0} {}

threadloop::skip_option offline_cam::_p_should_skip() {
    if (sensor_data_it_ != sensor_data_.end()) {
        dataset_now_ = sensor_data_it_->first;
        std::this_thread::sleep_for(
                time_point{std::chrono::nanoseconds{dataset_now_ - dataset_first_time_}} -
                clock_->now());
        return skip_option::run;
    } else {
        return skip_option::stop;
    }
}

void offline_cam::_p_one_iteration() {
    //auto start = std::chrono::high_resolution_clock::now();
    RAC_ERRNO_MSG("offline_cam at start of _p_one_iteration");
    assert(sensor_data_it_ != sensor_data_.end());
#ifndef NDEBUG
    std::chrono::time_point<std::chrono::nanoseconds> tp_dataset_now{
            std::chrono::nanoseconds{dataset_now_}};
    std::cerr << " CAM time: " << tp_dataset_now.time_since_epoch().count() << std::endl;
#endif
    const sensor_types& sensor_datum = sensor_data_it_->second;
    ++sensor_data_it_;

    auto cam0 = sensor_datum.cam0.load();
    RAC_ERRNO_MSG("offline_imu_cam after cam0");

    auto cam1 = sensor_datum.cam1.load();
    RAC_ERRNO_MSG("offline_imu_cam after cam1");

#ifndef NDEBUG
    /// If debugging, assert the image is grayscale
    const int num_ch0 = cam0->channels();
    const int num_ch1 = cam1->channels();
    assert(num_ch0 == 1 && "Data from lazy_load_image should be grayscale");
    assert(num_ch1 == 1 && "Data from lazy_load_image should be grayscale");
#endif /// NDEBUG
    cam_publisher_.put(cam_publisher_.allocate<data_format::binocular_cam_type>(data_format::binocular_cam_type{
            time_point{std::chrono::nanoseconds(dataset_now_ - dataset_first_time_)},
            cam0->clone(),
            cam1->clone()
    }));
}


PLUGIN_MAIN(offline_cam)
