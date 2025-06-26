#pragma once

#include "illixr/data_format/imu.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

namespace ILLIXR {
class android_imu : public threadloop {
public:
    [[maybe_unused]] android_imu(const std::string& name_, phonebook* pb_);

    ~android_imu() override;

private:
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<const RelativeClock> clock_;
    switchboard::writer<data_format::imu_type> imu_;

    static int android_sensor_callback(int fd, int events, void* data);

    static void* android_run_thread(void* ptr);

    /// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
    /// _p_one_iteration() is called in a thread created by threadloop::start()
    void _p_one_iteration() override;

};

}