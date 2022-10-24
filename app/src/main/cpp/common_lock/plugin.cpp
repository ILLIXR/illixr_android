//
// Created by madhuparna on 10/19/22.
//

#include "../common/common_lock.hpp"
#include "common/plugin.hpp"
#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "common-lock", __VA_ARGS__))

class common_lock_impl : public common_lock {
public:
    common_lock_impl(const phonebook* const pb)
            : sb{pb->lookup_impl<switchboard>()}
            , _m_clock{pb->lookup_impl<RelativeClock>()}
            , _m_slow_pose{sb->get_reader<pose_type>("slow_pose")}
            , _m_imu_raw{sb->get_reader<imu_raw_type>("imu_raw")}
            , _m_true_pose{sb->get_reader<pose_type>("true_pose")}
            , _m_ground_truth_offset{sb->get_reader<switchboard::event_wrapper<Eigen::Vector3f>>("ground_truth_offset")}
            , _m_vsync_estimate{sb->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")} { }

    void get_lock() {
        //lock.lock();
        while(my_lock != 0) { ;
        }
        my_lock = 1;
        LOGI("got lock ");
    }

    void release_lock() {
        my_lock = 0;
    }

private:
    std::atomic<int> my_lock = 0;
    mutable std::atomic<bool>                                        first_time{true};
    const std::shared_ptr<switchboard>                               sb;
    const std::shared_ptr<const RelativeClock>                       _m_clock;
    switchboard::reader<pose_type>                                   _m_slow_pose;
    switchboard::reader<imu_raw_type>                                _m_imu_raw;
    switchboard::reader<pose_type>                                   _m_true_pose;
    switchboard::reader<switchboard::event_wrapper<Eigen::Vector3f>> _m_ground_truth_offset;
    switchboard::reader<switchboard::event_wrapper<time_point>>      _m_vsync_estimate;
};

class common_lock_plugin : public plugin {
public:
    common_lock_plugin(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        pb->register_impl<common_lock>(
                std::static_pointer_cast<common_lock>(std::make_shared<common_lock_impl>(pb)));
    }
};

PLUGIN_MAIN(common_lock_plugin);