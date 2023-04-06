// Common parameters. Ultimately, these need to be moved to a yaml file.

#pragma once

#include <string>
#include <jni.h>
#include <android/native_activity.h>

using namespace ILLIXR;

class arcore_data {
public:
    JavaVM*     get_java_vm() {
        return this->vm;
    };
    void     set_java_vm(JavaVM* _vm) {
        this->vm = _vm;
    };

    JavaVM *vm;
};