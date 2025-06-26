#include "service.hpp"


log_service_impl::log_service_impl(const phonebook* const pb)
        : switchboard_{pb->lookup_impl<switchboard>()} {
    const char* illixr_data_c_str = std::getenv("ILLIXR_LOG");
    if (!illixr_data_c_str) {
        std::cerr << "Please define ILLIXR_LOG" << std::endl;
        ILLIXR::abort();
    }
    std::string illixr_data = std::string{illixr_data_c_str};
    log_file.open(illixr_data);
}

log_service_impl::~log_service_impl() {
    log_file.close();
}

void log_service_impl::write_duration(std::string plugin_name, double duration) {
    lock_.lock();
    log_file << plugin_name << " " << std::to_string(duration) << std::endl;
    lock_.unlock();
}

void log_service_impl::write_duration_filename(std::string plugin_name, double duration,
                                               std::string filename) {
    lock_.lock();
    log_file << plugin_name << " " << std::to_string(duration) << std::endl;
    lock_.unlock();
}


class log_service_plugin : public plugin {
public:
    [[maybe_unused]] log_service_plugin(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        pb->register_impl<log_service>(
                std::static_pointer_cast<log_service>(std::make_shared<log_service_impl>(pb)));
    }
};

PLUGIN_MAIN(log_service_plugin)
