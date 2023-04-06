#include <arcore_c_api.h>
#include "common/data_format.hpp"
#include "common/managed_thread.hpp"
#include "common/relative_clock.hpp"
#include "common/switchboard.hpp"
#include "common/threadloop.hpp"
#include "common/arcore.hpp"
#include <android/log.h>
#include <jni.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "arcore", __VA_ARGS__))

using namespace ILLIXR;

class arcore_plugin : public ILLIXR::threadloop {
public:
    arcore_plugin(std::string name_, phonebook* pb_)
            : threadloop{name_, pb_}
            , _m_rtc{pb->lookup_impl<RelativeClock>()} {

    }

    virtual void _p_one_iteration() override {
        if(!once) {

            ArSession *ar_session = NULL;
            //Get the JNI Env for the current thread
            JNIEnv *env = NULL;
//            jsize nVMs;
//            JNI_GetCreatedJavaVMs(NULL, 0, &nVMs); // 1. just get the required array length
//            JavaVM** buffer = new JavaVM*[nVMs];
//            JNI_GetCreatedJavaVMs(buffer, nVMs, &nVMs); // 2. get the data
            //ANativeActivity *activity = ILLIXR::android_activity;
            JavaVM *vm = NULL;// = ILLIXR::java_vm;

            if(vm == NULL) {
                LOGI("VM is NULL");
                once = true;
                return;
            }
            LOGI("vm IS NOT NULL");

            vm->AttachCurrentThread(&env, NULL);
            //Create AR session
            ArStatus status = ArSession_create(env, this, &ar_session);
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

            vm->DetachCurrentThread();
            once = true;
        }
    }

private:
    const std::shared_ptr<switchboard>             _m_sb;
    std::shared_ptr<RelativeClock> _m_rtc;
    bool once = false;
};

PLUGIN_MAIN(arcore_plugin)
