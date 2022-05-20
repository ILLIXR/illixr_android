//
// Created by madhuparna on 4/19/22.
//
#include <android/log.h>
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "sample", __VA_ARGS__))

int main() {

    LOGI("SAMPLE RUNNING");
    return 0;
}