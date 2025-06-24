//
// Created by madhuparna on 10/19/22.
//

#include "illixr/log_service.hpp"
#include "illixr/plugin.hpp"

class log_service_impl : public log_service {
public:
    log_service_impl(const phonebook* const pb)
            : sb{pb->lookup_impl<switchboard>()}{
        const char* illixr_data_c_str = std::getenv("ILLIXR_LOG");
        if (!illixr_data_c_str) {
            std::cerr << "Please define ILLIXR_LOG" << std::endl;
            ILLIXR::abort();
        }
        std::string illixr_data = std::string{illixr_data_c_str};
        log_file.open(illixr_data);
    }

    ~log_service_impl() {
        log_file.close();
    }

    void write_duration(std::string plugin_name, double duration) {
        lock.lock();
        log_file << plugin_name << " " << std::to_string(duration) <<std::endl;
        lock.unlock();
    }

    void write_duration_filename(std::string plugin_name, double duration, std::string filename) {
        lock.lock();
        log_file << plugin_name << " " << std::to_string(duration) <<std::endl;
        lock.unlock();
    }


private:
    const std::shared_ptr<switchboard>                               sb;
    std::mutex lock;
    std::vector<std::ofstream> files;
};

class log_service_plugin : public plugin {
public:
    log_service_plugin(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        pb->register_impl<log_service>(
                std::static_pointer_cast<log_service>(std::make_shared<log_service_impl>(pb)));
    }
};

PLUGIN_MAIN(log_service_plugin);