#include "plugin.hpp"
#include "illixr/global_module_defs.hpp"
#include <android/log.h>

#include <cassert>

#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "offline_imu_cam", __VA_ARGS__))

using namespace ILLIXR;

const record_header imu_cam_record{
        "imu_cam",
        {
                {"iteration_no", typeid(std::size_t)},
                {"has_camera", typeid(bool)},
        },
};

[[maybe_unused]] offline_imu_cam::offline_imu_cam(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , sensor_data_{load_data()}
        , sensor_data_it_{sensor_data_.cbegin()}
        , switchboard_{pb->lookup_impl<switchboard>()}
        , clock_{pb->lookup_impl<RelativeClock>()}
        , imu_cam_{switchboard_->get_writer<data_format::imu_cam_type>("imu_cam")}
        , dataset_first_time_{sensor_data_it_->first}, imu_cam_log_{record_logger_}
        , camera_cvtfmt_log_{record_logger_} {}

threadloop::skip_option offline_imu_cam::_p_should_skip() {
    if (sensor_data_it_ != sensor_data_.end()) {
        dataset_now_ = sensor_data_it_->first;

        std::this_thread::sleep_for(
                time_point{std::chrono::nanoseconds{dataset_now_ - dataset_first_time_}} -
                clock_->now());
        if (sensor_data_it_->second.imu0) {
            return skip_option::run;
        } else {
            ++sensor_data_it_;
            return skip_option::skip_and_yield;
        }

    } else {
        return skip_option::stop;
    }
}

void offline_imu_cam::_p_one_iteration() {
    auto start = std::chrono::high_resolution_clock::now();
    RAC_ERRNO_MSG("offline_imu_cam at start of _p_one_iteration");
    assert(sensor_data_it_ != sensor_data_.end());
#ifndef NDEBUG
    std::chrono::time_point<std::chrono::nanoseconds> tp_dataset_now{
            std::chrono::nanoseconds{dataset_now_}};
    std::cerr << " IMU time: " << tp_dataset_now.time_since_epoch().count() << std::endl;
#endif
    const sensor_types& sensor_datum = sensor_data_it_->second;
    ++sensor_data_it_;

    imu_cam_log_.log(record{imu_cam_record,
                            {
                                    {iteration_no},
                                    {bool(sensor_datum.cam0)},
                            }});

    std::optional<cv::Mat> cam0 =
            sensor_datum.cam0 ? std::make_optional<cv::Mat>(*(sensor_datum.cam0.value().load()))
                              : std::nullopt;
    RAC_ERRNO_MSG("offline_imu_cam after cam0");

    std::optional<cv::Mat> cam1 =
            sensor_datum.cam1 ? std::make_optional<cv::Mat>(*(sensor_datum.cam1.value().load()))
                              : std::nullopt;
    RAC_ERRNO_MSG("offline_imu_cam after cam1");

#ifndef NDEBUG
    /// If debugging, assert the image is grayscale
    if (cam0.has_value() && cam1.has_value()) {
        const int num_ch0 = cam0.value().channels();
        const int num_ch1 = cam1.value().channels();
        assert(num_ch0 == 1 && "Data from lazy_load_image should be grayscale");
        assert(num_ch1 == 1 && "Data from lazy_load_image should be grayscale");
    }
#endif /// NDEBUG

    imu_cam_.put(imu_cam_.allocate<data_format::imu_cam_type>(
            data_format::imu_cam_type{
                    time_point{std::chrono::nanoseconds(dataset_now_ - dataset_first_time_)},
                    (sensor_datum.imu0.value().angular_v).cast<float>(),
                    (sensor_datum.imu0.value().linear_a).cast<float>(), cam0, cam1}));
    ANDROID_LOG("AG: %f, %f, %f, %f, %f, %f", sensor_datum.imu0.value().angular_v[0],
                sensor_datum.imu0.value().angular_v[1], sensor_datum.imu0.value().angular_v[2],
                sensor_datum.imu0.value().linear_a[0], sensor_datum.imu0.value().linear_a[1],
                sensor_datum.imu0.value().linear_a[2]);

    RAC_ERRNO_MSG("offline_imu_cam at bottom of iteration");
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    ANDROID_LOG("duration: %f", duration2double(duration));
}

PLUGIN_MAIN(offline_imu_cam)
