#include "service.hpp"

#include <cassert>
#include <memory>
#include <android/log.h>

using namespace ILLIXR;
#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "extended-window", __VA_ARGS__))

xlib_gl_extended_window::xlib_gl_extended_window(int width, int height, EGLContext egl_context,
                                                 ANativeWindow* window) {
    my_window_ = window;
    width_ = width;
    height_ = height;
    display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major_version, minor_version;
    eglInitialize(display_, &major_version, &minor_version);
    ANDROID_LOG("EGL Initialized with major version : %d minor version: %d", major_version,
                minor_version);

    const EGLint attribs[] = {
            //EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_ALPHA_SIZE, 0,
            // EGL_DEPTH_SIZE, 24,
            EGL_NONE
    };

    EGLint w, h, format;
    EGLint numConfigs;
    EGLConfig config = nullptr;
    eglChooseConfig(display_, attribs, &config, 1, &numConfigs);
    std::unique_ptr<EGLConfig[]> supportedConfigs(new EGLConfig[numConfigs]);
    assert(supportedConfigs);
    eglChooseConfig(display_, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
    assert(numConfigs);
    auto i = 0;
    for (; i < numConfigs; i++) {
        auto& cfg = supportedConfigs[i];
        EGLint r, g, b, d;
        if (eglGetConfigAttrib(display_, cfg, EGL_RED_SIZE, &r) &&
            eglGetConfigAttrib(display_, cfg, EGL_GREEN_SIZE, &g) &&
            eglGetConfigAttrib(display_, cfg, EGL_BLUE_SIZE, &b) &&
            eglGetConfigAttrib(display_, cfg, EGL_DEPTH_SIZE, &d) &&
            r == 8 && g == 8 && b == 8 && d == 24) {

            config = supportedConfigs[i];
            break;
        }
    }
    if (i == numConfigs) {
        config = supportedConfigs[0];
    }

    if (config == nullptr) {
        return;
    }
    eglGetConfigAttrib(display_, config, EGL_NATIVE_VISUAL_ID, &format);
    if (window != nullptr) {
        ANDROID_LOG("window is not nullptr");
    } else {
        ANDROID_LOG("window is nullptr");
    }
    try {
        surface_ = eglCreateWindowSurface(display_, config, my_window_, nullptr);
    }
    catch (const std::exception& e) {
        return;
    }
    EGLint ctxattrb[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
    };
    context_ = eglCreateContext(display_, config, egl_context, ctxattrb);

    eglQuerySurface(display_, surface_, EGL_WIDTH, &w);
    eglQuerySurface(display_, surface_, EGL_HEIGHT, &h);
}

xlib_gl_extended_window::xlib_gl_extended_window(int width, int height, EGLContext egl_context) {
    width_ = width;
    height_ = height;
    display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major_version, minor_version;
    eglInitialize(display_, &major_version, &minor_version);
    ANDROID_LOG("EGL Initialized with major version : %d minor version: %d", major_version,
                minor_version);

    const EGLint attribs[] = {
            //EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_ALPHA_SIZE, 0,
            // EGL_DEPTH_SIZE, 24,
            EGL_NONE
    };

    //EGLint w, h;
    EGLint format;
    EGLint numConfigs;
    EGLConfig config = nullptr;
    eglChooseConfig(display_, attribs, &config, 1, &numConfigs);
    std::unique_ptr<EGLConfig[]> supportedConfigs(new EGLConfig[numConfigs]);
    assert(supportedConfigs);
    eglChooseConfig(display_, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
    assert(numConfigs);
    auto i = 0;
    for (; i < numConfigs; i++) {
        auto& cfg = supportedConfigs[i];
        EGLint r, g, b, d;
        if (eglGetConfigAttrib(display_, cfg, EGL_RED_SIZE, &r) &&
            eglGetConfigAttrib(display_, cfg, EGL_GREEN_SIZE, &g) &&
            eglGetConfigAttrib(display_, cfg, EGL_BLUE_SIZE, &b) &&
            eglGetConfigAttrib(display_, cfg, EGL_DEPTH_SIZE, &d) &&
            r == 8 && g == 8 && b == 8 && d == 24) {

            config = supportedConfigs[i];
            break;
        }
    }
    if (i == numConfigs) {
        config = supportedConfigs[0];
    }

    if (config == nullptr) {
        return;
    }
    eglGetConfigAttrib(display_, config, EGL_NATIVE_VISUAL_ID, &format);
//            if (window != nullptr) {
//                LOGI("window is not nullptr");
//            }
//            else{
//                LOGI("window is nullptr");
//            }
//            try {
//                surface = eglCreateWindowSurface(display, config, my_window, nullptr);
//            }
//            catch (const std::exception& e) {
//                return;
//            }
    surface_ = EGL_NO_SURFACE;
    EGLint ctxattrb[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
    };
    context_ = eglCreateContext(display_, config, egl_context, ctxattrb);

//            eglQuerySurface(display, surface, EGL_WIDTH, &w);
//            eglQuerySurface(display, surface, EGL_HEIGHT, &h);
}

xlib_gl_extended_window::~xlib_gl_extended_window() {
    if (display_ != EGL_NO_DISPLAY) {
        eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (context_ != EGL_NO_CONTEXT) {
            eglDestroyContext(display_, context_);
        }
        if (surface_ != EGL_NO_SURFACE) {
            eglDestroySurface(display_, surface_);
        }
        eglTerminate(display_);
    }
    display_ = EGL_NO_DISPLAY;
    context_ = EGL_NO_CONTEXT;
    surface_ = EGL_NO_SURFACE;
}

