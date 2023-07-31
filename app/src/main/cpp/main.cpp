#include "common/extended_window.hpp"
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/log.h>
#include <thread>
#include <vector>
#include "runtime/main.h"

using namespace ILLIXR;

static void handle_cmd(struct android_app* app, int32_t cmd) {
    switch(cmd) {
        case APP_CMD_INIT_WINDOW:
        {

//          GlDemo with Pose Lookup
//          std::vector<std::string> arguments = { "", "libpose_lookup.so", "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};

//          GlDemo with Offline IMU Cam (EuRoC Dataset)
          std::vector<std::string> arguments = { "", "libslam.so", "liboffline_imu_cam.so" ,"librk4_integrator.so",
                                                  "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};

//          GlDemo with Online IMU and Cam
//            std::vector<std::string> arguments = { "", "libslam.so", "libandroid_imu_cam.so" ,"librk4_integrator.so",
//                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
            std::vector<char*> argv;
            for (const auto& arg : arguments)
                argv.push_back((char*)arg.data());

            //Change this to application path on the device
            std::string app_path = "/sdcard/Android/data/com.example.native_activity/";

            setenv("ILLIXR_DATA", (app_path + "/mav0").c_str(), true);
            setenv("ILLIXR_LOG", (app_path + "/log.txt").c_str(), true);
            setenv("ILLIXR_DEMO_DATA", (app_path + "/demo_data").c_str(), true);

            setenv("ILLIXR_OFFLOAD_ENABLE", "False", true);
            setenv("ILLIXR_ALIGNMENT_ENABLE", "False", true);
            setenv("ILLIXR_ENABLE_VERBOSE_ERRORS", "False", true);
            setenv("ILLIXR_RUN_DURATION", "120", true);
            setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
            setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
            std::thread runtime_thread(runtime_main, argv.size(), argv.data(), app->window);
            runtime_thread.join();
            break;
        }
        default: ;
    }
}


void android_main(struct android_app* state) {
    state->onAppCmd = handle_cmd;
    while(true) {
        int ident;
        int events;
        struct android_poll_source* source;

        // If not animating, we will block forever waiting for events.
        // If animating, we loop until all events are read, then continue
        // to draw the next frame of animation.
        while ((ident=ALooper_pollAll(0, nullptr, &events,
                                      (void**)&source)) >= 0) {

            // Process this event.
            if (source != nullptr) {
                source->process(state, source);
            }
        }
    }
}