#include "extended_window.hpp"
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/log.h>
#include <thread>
#include <vector>
#include "runtime/main.h"
//#define  ILLIXR_MONADO 1
using namespace ILLIXR;

#define LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android-main", __VA_ARGS__))

extern "C" {
    // called from Java after permission is granted
    JNIEXPORT void JNICALL Java_com_example_ILLIXR_ILLIXRNativeActivity_nativeOnPermissionGranted(JNIEnv* env, jobject activity) {

    }
}
static void handle_cmd(struct android_app* app, int32_t cmd) {
    switch(cmd) {
        case APP_CMD_INIT_WINDOW:
        {
//            std::vector<std::string> arguments = { "", "libandroid_imu_cam.so"};
//            std::vector<std::string> arguments = { "", "liblog_service.so", "libslam.so" ,"librk4_integrator.so",
//                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
//            std::vector<std::string> arguments = { "", "liblog_service.so",
//                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
            std::vector<std::string> arguments = { "", "liblog_service.so", "libslam.so", "libandroid_imu_cam.so" ,"librk4_integrator.so",
                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
            std::vector<char*> argv;
            for (const auto& arg : arguments)
                argv.push_back((char*)arg.data());
            //EuRoC
            setenv("ILLIXR_DATA", "/sdcard/Android/data/com.example.native_activity/android_new", true);
            setenv("ILLIXR_LOG", "/sdcard/Android/data/com.example.native_activity/log.txt", true);

            //Android
//            setenv("ILLIXR_DATA", "/sdcard/Android/data/com.example.native_activity/android_new", true);
            setenv("ILLIXR_DEMO_DATA", "/sdcard/Android/data/com.example.native_activity/demo_data", true);
            setenv("ILLIXR_OFFLOAD_ENABLE", "False", true);
            setenv("ILLIXR_ALIGNMENT_ENABLE", "False", true);
            setenv("ILLIXR_ENABLE_VERBOSE_ERRORS", "False", true);
            setenv("ILLIXR_RUN_DURATION", "1000000", true);
            setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
            setenv("ILLIXR_ENABLE_PRE_SLEEP", "False", true);
            std::thread runtime_thread(runtime_main, argv.size(), argv.data(), app->window);
            runtime_thread.join();
            break;
        }
        default:
            LOG("Some other command");
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
        do {
            ident = ALooper_pollOnce(0, nullptr, &events,
                                     (void**)&source);
            if (ident >= 0) {
                // Process this event.
                if (source != nullptr) {
                    source->process(state, source);
                }
            }
        } while (ident >= 0);
    }
}