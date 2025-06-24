#pragma once

#include "illixr/switchboard.hpp"
#include <opencv2/core/mat.hpp>

namespace ILLIXR::data_format {

    struct cam_type : switchboard::event {
        time_point time;
        cv::Mat    img0;
        cv::Mat    img1;

        cam_type(time_point _time, cv::Mat _img0, cv::Mat _img1)
                : time{_time}
                , img0{_img0}
                , img1{_img1} { }
    };

    class rgb_depth_type : public switchboard::event {
        [[maybe_unused]] time_point time;
        std::optional<cv::Mat>      rgb;
        std::optional<cv::Mat>      depth;

    public:
        rgb_depth_type(time_point _time,
                       std::optional<cv::Mat> _rgb,
                       std::optional<cv::Mat> _depth
        )
                : time{_time}
                , rgb{_rgb}
                , depth{_depth}
        { }
    };

}
