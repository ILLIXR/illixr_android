#pragma once

#include "phonebook.hpp"
#include <fstream>

using namespace ILLIXR;

class log_service : public phonebook::service {
public:
    std::ofstream log_file;
    virtual void  write_duration(std::string plugin_name, double duration)    = 0;
    virtual void  write_duration_filename(std::string plugin_name, double duration, std::string filename)    = 0;
    virtual ~log_service() { }
};
