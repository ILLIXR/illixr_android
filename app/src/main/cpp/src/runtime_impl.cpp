#include "illixr/runtime.hpp"

#include "illixr/dynamic_lib.hpp"
#include "illixr/error_util.hpp"
#include "illixr/extended_window.hpp"
#include "illixr/global_module_defs.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/plugin.hpp"
#include "illixr/record_logger.hpp"
#include "illixr/stoplight.hpp"
#include "illixr/switchboard.hpp"

#include "noop_record_logger.hpp"
//#include "sqlite_record_logger.hpp"
//#include "stdout_record_logger.hpp"

#include <algorithm>
#include <memory>
//#include <set>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/android_sink.h>
#include <string>
#include <vector>

using namespace ILLIXR;

void spdlogger(const std::string& name) {
#ifdef NDEBUG
    const std::string log_level = "warn";
#else
    const std::string log_level = "debug";
#endif
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink_mt>());
    auto logger = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
    logger->set_level(spdlog::level::from_str(log_level));
    spdlog::register_logger(logger);
}

class runtime_impl : public runtime {
public:
    runtime_impl(
#ifndef ENABLE_MONADO
            EGLContext appGLCtx,
            ANativeWindow *window
#endif
    ) {
        spdlogger("illixr");
        phonebook_.register_impl<record_logger>(std::make_shared<noop_record_logger>());
        phonebook_.register_impl<switchboard>(std::make_shared<switchboard>(&phonebook_));
#ifndef ENABLE_MONADO
        phonebook_.register_impl<xlib_gl_extended_window>(
                std::make_shared<xlib_gl_extended_window>(display_params::width_pixels, display_params::height_pixels, appGLCtx, window));
#endif /// ENABLE_MONADO
        switchboard_   = phonebook_.lookup_impl<switchboard>();
        phonebook_.register_impl<stoplight>(std::make_shared<stoplight>());
        phonebook_.register_impl<relative_clock>(std::make_shared<relative_clock>());
    }

    void load_so(const std::vector<std::string>& so_paths) override {
        RAC_ERRNO_MSG("runtime_impl before creating any dynamic library");

        std::transform(so_paths.cbegin(), so_paths.cend(), std::back_inserter(libraries_), [](const auto& so_path) {
            RAC_ERRNO_MSG("runtime_impl before creating the dynamic library");
            return dynamic_lib::create(so_path);
        });

        RAC_ERRNO_MSG("runtime_impl after creating the dynamic libraries");

        std::vector<plugin_factory> plugin_factories;
        std::transform(libraries_.cbegin(), libraries_.cend(), std::back_inserter(plugin_factories), [](const auto& lib) {
            return lib.template get<plugin* (*) (phonebook*)>("this_plugin_factory");
        });

        RAC_ERRNO_MSG("runtime_impl after generating plugin factories");
        phonebook_.lookup_impl<relative_clock>()->start();

        std::transform(plugin_factories.cbegin(), plugin_factories.cend(), std::back_inserter(plugins_),
                       [this](const auto& plugin_factory) {
                           RAC_ERRNO_MSG("runtime_impl before building the plugin");
                           try {
                               return std::unique_ptr<plugin>{plugin_factory(&phonebook_)};
                           } catch (std::exception& ex) {
                               spdlog::get("illixr")->error(ex.what());
                               throw;
                           }
                       });

        std::for_each(plugins_.cbegin(), plugins_.cend(), [](const auto& plugin) {
            // Well-behaved plugins_ (any derived from threadloop) start there threads here, and then wait on the Stoplight.
            plugin->start();
        });

        // This actually kicks off the plugins
        phonebook_.lookup_impl<stoplight>()->signal_ready();
    }

    void load_so(const std::string_view& so) override {
        auto lib                 = dynamic_lib::create(so);
        auto this_plugin_factory = lib.get<plugin* (*) (phonebook*)>("this_plugin_factory");
        load_plugin_factory(this_plugin_factory);
        libraries_.push_back(std::move(lib));
    }

    void load_plugin_factory(plugin_factory plugin_main) override {
        plugins_.emplace_back(plugin_main(&phonebook_));
        plugins_.back()->start();
    }

    void wait() override {
        // We don't want wait() returning before all the plugin threads have been joined.
        // That would cause a nasty race-condition if the client tried to delete the runtime right after wait() returned.
        phonebook_.lookup_impl<stoplight>()->wait_for_shutdown_complete();
    }

    void _stop() override {
        phonebook_.lookup_impl<stoplight>()->signal_should_stop();
        // After this point, threads may exit their main loops
        // They still have destructors and still have to be joined.

        phonebook_.lookup_impl<switchboard>()->stop();
        // After this point, Switchboard's internal thread-workers which power synchronous callbacks are stopped and joined.

        for (const std::shared_ptr<plugin>& plugin : plugins_) {
            plugin->stop();
            // Each plugin gets joined in its stop
        }

        // Tell runtime::wait() that it can return
        phonebook_.lookup_impl<stoplight>()->signal_shutdown_complete();
    }

    ~runtime_impl() override {
        if (!phonebook_.lookup_impl<stoplight>()->check_shutdown_complete()) {
            stop();
        }
        // This will be re-enabled in #225
        // assert(errno == 0 && "errno was set during run. Maybe spurious?");
        /*
          Note that this assertion can have false positives AND false negatives!
          - False positive because the contract of some functions specifies that errno is only meaningful if the return code was
          an error [1].
            - We will try to mitigate this by clearing errno on success in ILLIXR.
          - False negative if some intervening call clears errno.
            - We will try to mitigate this by checking for errors immediately after a call.

          Despite these mitigations, the best way to catch errors is to check errno immediately after a calling function.

          [1] https://cboard.cprogramming.com/linux-programming/119957-xlib-perversity.html
         */
    }

private:
    // I have to keep the dynamic libraries in scope until the program is dead
    std::vector<dynamic_lib>             libraries_;
    phonebook                            phonebook_;
    std::vector<std::shared_ptr<plugin>> plugins_;
};


#ifdef ENABLE_MONADO
extern "C" runtime* runtime_factory() {
        RAC_ERRNO_MSG("runtime_impl before creating the runtime");
        return new runtime_impl{};
    }
#else
extern "C" runtime* runtime_factory(EGLContext appGLCtx, ANativeWindow *window) {
    RAC_ERRNO_MSG("runtime_impl before creating the runtime");
    return new runtime_impl{appGLCtx, window};
}
#endif /// ENABLE_MONADO
