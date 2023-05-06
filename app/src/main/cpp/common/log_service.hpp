//
// Created by madhuparna on 10/19/22.
//

#ifndef ILLIXR_NATIVE_ACTIVITY_LOG_SERVICE_HPP
#define ILLIXR_NATIVE_ACTIVITY_LOG_SERVICE_HPP

#endif //ILLIXR_NATIVE_ACTIVITY_LOG_SERVICE_HPP

#include "data_format.hpp"
#include "phonebook.hpp"
#include <fstream>

using namespace ILLIXR;

class log_service : public phonebook::service {
public:
    std::ofstream log_file;
    virtual void  write_duration(std::string plugin_name, double duration)    = 0;
    virtual ~log_service() { }
};
