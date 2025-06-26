#include "plugin.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <shared_mutex>

using namespace ILLIXR;

[[maybe_unused]] offline_cam::offline_cam(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , switchboard_{pb->lookup_impl<switchboard>()}
        , cam_publisher_{switchboard_->get_writer<data_format::cam_type>("cam")}
        , sensor_data_{load_data()}
        , dataset_first_time_{sensor_data_.cbegin()->first}
        , last_ts_{0}
        , clock_{pb->lookup_impl<RelativeClock>()}
        , next_row_{sensor_data_.cbegin()} {}

threadloop::skip_option offline_cam::_p_should_skip() {
    if (true) {
        return skip_option::run;
    } else {
        return skip_option::stop;
    }
}

void offline_cam::_p_one_iteration() {
    duration time_since_start = clock_->now().time_since_epoch();
    // duration begin            = time_since_start;
    ullong lookup_time = std::chrono::nanoseconds{time_since_start}.count() + dataset_first_time_;
    std::map<ullong, sensor_types>::const_iterator nearest_row;

    // "std::map::upper_bound" returns an iterator to the first pair whose key is GREATER than the argument.
    auto after_nearest_row = sensor_data_.upper_bound(lookup_time);

    if (after_nearest_row == sensor_data_.cend()) {
#ifndef NDEBUG
        std::cerr << "Running out of the dataset! "
                  << "Time " << lookup_time << " (" << clock_->now().time_since_epoch().count()
                  << " + "
                  << dataset_first_time_ << ") after last datum " << sensor_data_.rbegin()->first
                  << std::endl;
#endif
        // Handling the last camera images. There's no more rows after the nearest_row, so we set after_nearest_row
        // to be nearest_row to avoiding sleeping at the end.
        nearest_row = std::prev(after_nearest_row, 1);
        after_nearest_row = nearest_row;
        // We are running out of the dataset and the loop will stop next time.
        internal_stop();
    } else if (after_nearest_row == sensor_data_.cbegin()) {
        // Should not happen because lookup_time is bigger than dataset_first_time
#ifndef NDEBUG
        std::cerr << "Time " << lookup_time << " (" << clock_->now().time_since_epoch().count()
                  << " + "
                  << dataset_first_time_ << ") before first datum " << sensor_data_.cbegin()->first
                  << std::endl;
#endif
    } else {
        // Most recent
        nearest_row = std::prev(after_nearest_row, 1);
    }

    if (last_ts_ != nearest_row->first) {
        last_ts_ = nearest_row->first;

        cv::Mat img0 = nearest_row->second.cam0.load();
        cv::Mat img1 = nearest_row->second.cam1.load();

        time_point expected_real_time_given_dataset_time(
                std::chrono::duration<long, std::nano>{nearest_row->first - dataset_first_time_});
        cam_publisher_.put(cam_publisher_.allocate<data_format::cam_type>(data_format::cam_type{
                expected_real_time_given_dataset_time,
                img0,
                img1,
        }));
    }
    std::this_thread::sleep_for(
            std::chrono::nanoseconds(after_nearest_row->first - dataset_first_time_ -
                                     clock_->now().time_since_epoch().count() - 2));
}

PLUGIN_MAIN(offline_cam)
