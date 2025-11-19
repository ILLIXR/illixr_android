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


#ifdef UNITY_LIBRARY

extern "C" {
    void initialize_for_unity(const char* path, TextCallback callback);
}
#endif

namespace ILLIXR {
    void run(const std::vector<std::string>& plugins, TextCallback callback);
} // namespace ILLIXR
