#pragma once

#include "illixr/runtime.hpp"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <android/native_window_jni.h>

constexpr std::chrono::seconds          ILLIXR_RUN_DURATION_DEFAULT{60};
[[maybe_unused]] constexpr unsigned int ILLIXR_PRE_SLEEP_DURATION{10};

extern ILLIXR::runtime* runtime_;

namespace ILLIXR {
int run(const std::vector<std::string>& plugins, ANativeWindow *window);

class cancellable_sleep {
public:
    template<typename T, typename R>
    bool sleep(std::chrono::duration<T, R> duration) {
        auto wake_up_time = std::chrono::system_clock::now() + duration;
        while (!terminate_.load() && std::chrono::system_clock::now() < wake_up_time) {
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
        return terminate_.load();
    }

    void cancel() {
        terminate_.store(true);
    }

private:
    std::atomic<bool> terminate_{false};
};
} // namespace ILLIXR
