#pragma once

#include "illixr/data_format/misc.hpp"
#include "illixr/csv_iterator.hpp"
#include "illixr/error_util.hpp"

#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <optional>
#include <string>


/*
 * Uncommenting this preprocessor macro makes the offline_cam load each data from the disk as it is needed.
 * Otherwise, we load all of them at the beginning, hold them in memory, and drop them in the queue as needed.
 * Lazy loading has an artificial negative impact on performance which is absent from an online-sensor system.
 * Eager loading deteriorates the startup time and uses more memory.
 */
//#define LAZY

class lazy_load_image {
public:
    lazy_load_image(const std::string& path)
        : _m_path(path) {
    }

    std::unique_ptr<cv::Mat> load() const {
        auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_GRAYSCALE)}};
        assert(!img->empty());
        return img;
    }

private:
    std::string _m_path;
};

typedef struct {
    lazy_load_image cam0;
    lazy_load_image cam1;
} sensor_types;

[[maybe_unused]] static std::map<ullong, sensor_types> load_data() {
    const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
    if (!illixr_data_c_str) {
        std::cerr << "Please define ILLIXR_DATA" << std::endl;
        ILLIXR::abort();
    }
    std::string illixr_data = std::string{illixr_data_c_str};

    std::map<ullong, sensor_types> data;

    const std::string cam0_subpath = "/cam0/data.csv";
    std::ifstream     cam0_file{illixr_data + cam0_subpath};
    if (!cam0_file.good()) {
        std::cerr << "${ILLIXR_DATA}" << cam0_subpath << " (" << illixr_data << cam0_subpath << ") is not a good path"
                  << std::endl;
        ILLIXR::abort();
    }
    for (CSVIterator row{cam0_file, 1}; row != CSVIterator{}; ++row) {
        ullong t     = std::stoull(row[0]);
        data[t].cam0 = {illixr_data + "/cam0/data/" + row[1]};
    }

    const std::string cam1_subpath = "/cam1/data.csv";
    std::ifstream     cam1_file{illixr_data + cam1_subpath};
    if (!cam1_file.good()) {
        std::cerr << "${ILLIXR_DATA}" << cam1_subpath << " (" << illixr_data << cam1_subpath << ") is not a good path"
                  << std::endl;
        ILLIXR::abort();
    }
    for (CSVIterator row{cam1_file, 1}; row != CSVIterator{}; ++row) {
        ullong      t     = std::stoull(row[0]);
        std::string fname = row[1];
        data[t].cam1      = {illixr_data + "/cam1/data/" + row[1]};
    }

    return data;
}
