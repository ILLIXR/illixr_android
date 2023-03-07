#include <opencv2/opencv.hpp> // Include OpenCV API
#include <opencv2/core/mat.hpp>

// ILLIXR includes
#include "common/data_format.hpp"
#include "common/switchboard.hpp"
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <android/log.h>

#define LOGA(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_cam", __VA_ARGS__))

using namespace ILLIXR;

class android_cam : public threadloop {
public:
    android_cam(std::string name_, phonebook* pb_)
            : threadloop{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , _m_clock{pb->lookup_impl<RelativeClock>()}
            , _m_cam{sb->get_writer<cam_type>("cam")}{
    }

    virtual ~android_cam() override {
        std::cout << "Deconstructing android_cam" << std::endl;
    }

    /// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
    /// _p_one_iteration() is called in a thread created by threadloop::start()
    void _p_one_iteration() override {
        LOGA("Android_cam started ..");
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
        if (!_m_clock->is_started()) {
            return;
        }
        double ts = std::chrono::system_clock::now().time_since_epoch().count();;
        ullong cam_time = static_cast<ullong>(ts * 1000000);
        if (!_m_first_cam_time) {
            _m_first_cam_time      = cam_time;
            _m_first_real_time_cam = _m_clock->now();
        }
        cv::Mat ir_left = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
        cv::Mat ir_right = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
        time_point cam_time_point{*_m_first_real_time_cam + std::chrono::nanoseconds(cam_time - *_m_first_cam_time)};
        LOGA("Writing to cam topic ..");
        _m_cam.put(_m_cam.allocate<cam_type>({cam_time_point, ir_left, ir_right}));
        LOGA("Done writing cam ..");
    }

private:
    const std::shared_ptr<switchboard>         sb;
    const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<cam_type>              _m_cam;
    std::mutex                                 mutex;

    std::optional<ullong>     _m_first_cam_time;
    std::optional<time_point> _m_first_real_time_cam;

};

PLUGIN_MAIN(android_cam);

