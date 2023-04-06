#include "extended_window.hpp"
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/log.h>
#include <thread>
#include <vector>
#include "runtime/main.h"
#include "../../../../arcore-android-sdk/libraries/include/arcore_c_api.h"
#include "common/arcore.hpp"

#define  ILLIXR_MONADO 0
using namespace ILLIXR;

#define LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android-main", __VA_ARGS__))

void arcore_get_pose(ANativeActivity* activity) {
    ArSession *ar_session = NULL;
    //Get the JNI Env for the current thread
    JNIEnv *env = NULL;
    JavaVM *vm = activity->vm;
    vm->AttachCurrentThread(&env, NULL);
    //Create AR session
    ArStatus status = ArSession_create(env, activity->clazz, &ar_session);
    if (status == ArStatus::AR_SUCCESS)
        LOGI("Success");
    else
        LOGI("Failure ");


    // Create a session config.
    ArConfig *ar_config = NULL;
    ArConfig_create(ar_session, &ar_config);
    if (ArSession_configure(ar_session, ar_config) == AR_SUCCESS)
        LOGI("Session configure success");
    else
        LOGI("Session failure");
    ArFrame *ar_frame = NULL;
    ArFrame_create(ar_session, &ar_frame);
    ArSession_update(ar_session, ar_frame);
    ArPose *ar_pose = NULL;
    ArFrame_getAndroidSensorPose(ar_session, ar_frame, ar_pose);
    float raw_pose[7];
    ArPose_getPoseRaw(ar_session, ar_pose, raw_pose);
    LOGI("AR Pose : %f %f %f %f %f %f %f", raw_pose[0], raw_pose[1], raw_pose[2], raw_pose[3], raw_pose[4], raw_pose[5], raw_pose[6]);
    vm->DetachCurrentThread();
}
static void handle_cmd(struct android_app* app, int32_t cmd) {
    switch(cmd) {
        case APP_CMD_INIT_WINDOW:
        {
            std::vector<std::string> arguments = { "", "libslam.so" ,"librk4_integrator.so",
                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
//            std::vector<std::string> arguments = { "", "libslam.so", "libandroid_cam.so", "libandroid_imu.so" ,"librk4_integrator.so",
//                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};
//            std::vector<std::string> arguments = { "", "libslam.so", "liboffline_cam.so", "liboffline_imu.so" ,"librk4_integrator.so",
//                                                   "libpose_prediction.so",  "libcommon_lock.so", "libtimewarp_gl.so", "libgldemo.so"};

           // ArInstallStatus install_status;
            // === ATTENTION!  ATTENTION!  ATTENTION! ===
            // This method can and will fail in user-facing situations.  Your
            // application must handle these cases at least somewhat gracefully.  See
            // HelloAR Java sample code for reasonable behavior.

            //            ArAvailability availability;
//            ArCoreApk_checkAvailability(app->activity->env, app->activity->clazz, &availability);
//            if (availability == AR_AVAILABILITY_UNKNOWN_CHECKING) {
//                // Set a timer to call maybeEnableArButton() again after about 200ms.
//            }
//            if (availability == AR_AVAILABILITY_SUPPORTED_NOT_INSTALLED ||
//                availability == AR_AVAILABILITY_SUPPORTED_APK_TOO_OLD ||
//                availability == AR_AVAILABILITY_SUPPORTED_INSTALLED) {
//                // Show or enable the AR button.
//            } else {
//                // Hide or disable the AR button.
//            }
                arcore_get_pose(app->activity);
//            ILLIXR::android_activity = app->activity;
//            ILLIXR::java_vm = app->activity->vm;
            //Get the Android activity reference by a global variable recorded in glue call-back function

//            ArCoreApk_requestInstall(app->activity->env, app->activity->clazz, 1,&install_status) ;
//
//            switch (install_status) {
//                case AR_INSTALL_STATUS_INSTALLED:
//                    break;
//                case AR_INSTALL_STATUS_INSTALL_REQUESTED:
//                    return;
//            }

            // Likely called from Activity.onCreate() of an activity with AR buttons.


            std::vector<char*> argv;
            for (const auto& arg : arguments)
                argv.push_back((char*)arg.data());
            setenv("ILLIXR_DATA", "/sdcard/Android/data/com.example.native_activity/mav0", true);
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
        case APP_CMD_RESUME: {
        };
        default:
            LOG("Some other command");
    }
}


void android_main(struct android_app* state) {

    state->onAppCmd = handle_cmd;
    // Create a new ARCore session.

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