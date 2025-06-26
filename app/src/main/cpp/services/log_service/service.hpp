#pragma once

#include "illixr/log_service.hpp"
#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"

namespace ILLIXR {
class log_service_impl : public log_service {
public:
    explicit log_service_impl(const phonebook* const pb);

    ~log_service_impl() override;

    void write_duration(std::string plugin_name, double duration) override;

    void write_duration_filename(std::string plugin_name, double duration,
                                 std::string filename) override;

private:
    const std::shared_ptr<switchboard> switchboard_;
    std::mutex lock_;
};

}