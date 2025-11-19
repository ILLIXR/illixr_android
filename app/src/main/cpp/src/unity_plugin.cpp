#include "illixr.hpp"
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <csignal>
#include <unistd.h> /// Not portable

#define _STR(y)      #y
#define STRINGIZE(x) _STR(x)

ILLIXR::runtime* runtime_ = nullptr;
#ifdef UNITY_LIBRARY

extern "C" {
#endif
void initialize_for_unity(const char* path, TextCallback callback) {
    std::string data_path;
    if (path != nullptr) {
        data_path = std::string(path);
    }
    const std::vector <std::string> plugins = {"unity.pose"};//, "common_lock", "timewarp_gl",
                                               //"gldemo"};

    //EuRoC
    //setenv("ILLIXR_DATA", "/sdcard/Android/data/com.example.native_activity/mav0", true);
    //setenv("ILLIXR_LOG", "/sdcard/Android/data/com.example.native_activity/log.txt", true);

    //Android
//            setenv("ILLIXR_DATA", "/sdcard/Android/data/com.example.native_activity/android_new", true);
    setenv("ILLIXR_DEMO_DATA", data_path.c_str(), true);
    setenv("ILLIXR_OFFLOAD_ENABLE", "False", true);
    setenv("ILLIXR_ALIGNMENT_ENABLE", "False", true);
    setenv("ILLIXR_ENABLE_VERBOSE_ERRORS", "False", true);
    setenv("ILLIXR_RUN_DURATION", "1000000", true);
    setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
    setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
    //setenv("ILLIXR_TCP_CLIENT_IP", "141.142.60.47", true);
    //setenv("ILLIXR_TCP_CLIENT_PORT", "9000", true);
    //setenv("ILLIXR_TCP_SERVER_IP", "141.142.60.195", true);
    //setenv("ILLIXR_TCP_SERVER_PORT", "9001", true);
    //setenv("ILLIXR_IS_CLIENT", "1", true);

    std::thread runtime_thread(ILLIXR::run, plugins, callback);
    runtime_thread.detach();
}
#ifdef UNITY_LIBRARY
}
#endif
void ILLIXR::run(const std::vector<std::string>& plugins, TextCallback callback) {
    try {
        runtime_ = ILLIXR::runtime_factory(callback);

// set internal env_vars
        std::shared_ptr<ILLIXR::switchboard> switchboard_ = runtime_->get_switchboard();

        RAC_ERRNO_MSG("main after creating runtime");

        std::vector<std::string> lib_paths;
        std::transform(plugins.begin(), plugins.end(), std::back_inserter(lib_paths), [](const std::string& arg) {
            return "libplugin." + arg + STRINGIZE(ILLIXR_BUILD_SUFFIX) + ".so";
        });
        RAC_ERRNO_MSG("main before loading dynamic libraries");
        runtime_->load_so(lib_paths);

        runtime_->wait(); // blocks until shutdown is r->stop()

        delete runtime_;
    } catch (const std::exception& ex) {
        delete runtime_;
    }
}
