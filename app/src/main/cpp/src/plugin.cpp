#include "illixr.hpp"

#include <EGL/egl.h>
#include <csignal>
#include <unistd.h> /// Not portable

#define _STR(y)      #y
#define STRINGIZE(x) _STR(x)

ILLIXR::runtime* runtime_ = nullptr;

using namespace ILLIXR;

int ILLIXR::run(const std::vector<std::string>& plugins, ANativeWindow* window) {
    std::chrono::seconds     run_duration;
    try {

#ifdef ENABLE_MONADO
        r = ILLIXR::runtime_factory();
#else
        runtime_ = ILLIXR::runtime_factory(EGL_NO_CONTEXT, window);
#endif /// ENABLE_MONADO

// set internal env_vars
        std::shared_ptr<switchboard> switchboard_ = runtime_->get_switchboard();
#ifndef NDEBUG
        /// Activate sleeping at application start for attaching gdb. Disables 'catchsegv'.
        /// Enable using the ILLIXR_ENABLE_PRE_SLEEP environment variable (see 'runner/runner/main.py:load_tests')
        const bool enable_pre_sleep = switchboard_->get_env_bool("ILLIXR_ENABLE_PRE_SLEEP", "False");
        if (enable_pre_sleep) {
            const pid_t pid = getpid();
            spdlog::get("illixr")->info("[main] Pre-sleep enabled.");
            spdlog::get("illixr")->info("[main] PID: {}", pid);
            spdlog::get("illixr")->info("[main] Sleeping for {} seconds...", ILLIXR_PRE_SLEEP_DURATION);
            sleep(ILLIXR_PRE_SLEEP_DURATION);
            spdlog::get("illixr")->info("[main] Resuming...");
        }
#endif /// NDEBUG

        /// Shutting down method 2: Run timer
        run_duration = (!switchboard_->get_env("ILLIXR_RUN_DURATION").empty())
                       ? std::chrono::seconds{std::stol(std::string{switchboard_->get_env("ILLIXR_RUN_DURATION")})}
                       : ILLIXR_RUN_DURATION_DEFAULT;

        RAC_ERRNO_MSG("main after creating runtime");

        std::vector<std::string> lib_paths;
        std::transform(plugins.begin(), plugins.end(), std::back_inserter(lib_paths), [](const std::string& arg) {
            return "libplugin." + arg + STRINGIZE(ILLIXR_BUILD_SUFFIX) + ".so";
        });
        RAC_ERRNO_MSG("main before loading dynamic libraries");
        runtime_->load_so(lib_paths);

        cancellable_sleep cs;
        std::thread th{[&] {
            cs.sleep(run_duration);
            runtime_->stop();
        }};

        runtime_->wait(); // blocks until shutdown is r->stop()

        // cancel our sleep, so we can join the other thread
        cs.cancel();
        th.join();

        delete runtime_;
    } catch (const std::exception& ex) {
        delete runtime_;
    }
    return 0;
}
