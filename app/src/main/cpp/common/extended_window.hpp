

#pragma once

#include <cerrno>
#include <cassert>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include "phonebook.hpp"
#include "global_module_defs.hpp"
//#include "error_util.hpp"
#include <cerrno>
#include <cassert>
#include <initializer_list>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <jni.h>
#include <android/log.h>
#include <android/native_window.h>

//typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "extended-window", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "extended-wondow", __VA_ARGS__))

namespace ILLIXR {
    class xlib_gl_extended_window : public phonebook::service {
    public:
        int width;
        int height;
        EGLDisplay display;
        EGLSurface surface;
        EGLContext context;

        xlib_gl_extended_window(int _width, int _height, EGLContext egl_context, ANativeWindow *window) {
            LOGI("constructor started");

            width = _width;
            height = _height;

            /*
           #ifndef NDEBUG
                       std::cout << "Opening display" << std::endl;
           #endif
                       RAC_ERRNO_MSG("extended_window at start of xlib_gl_extended_window constructor");
           */
            //dpy = XOpenDisplay(nullptr);
            display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            LOGI("Display obtained");
            eglInitialize(display, nullptr, nullptr);
            LOGI("EGL Initialized");
            /* if (display == nullptr) {
                 ILLIXR::abort("Cannot connect");
             } else {
                 // Apparently, XOpenDisplay's _true_ error indication is whether dpy is nullptr.
                 // https://cboard.cprogramming.com/linux-programming/119957-xlib-perversity.html
                 // if (errno != 0) {
                 // 	std::cerr << "XOpenDisplay succeeded, but errno = " << errno << "; This is benign, so I'm clearing it now.\n";
                 // }
                 errno = 0;
             }*/

            //Window root = DefaultRootWindow(dpy);
            // Get a matching FB config
            /*static int visual_attribs[] =
                    {
                            GLX_X_RENDERABLE    , True,
                            GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
                            GLX_RENDER_TYPE     , GLX_RGBA_BIT,
                            GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
                            GLX_RED_SIZE        , 8,
                            GLX_GREEN_SIZE      , 8,
                            GLX_BLUE_SIZE       , 8,
                            GLX_ALPHA_SIZE      , 8,
                            GLX_DEPTH_SIZE      , 24,
                            GLX_STENCIL_SIZE    , 8,
                            GLX_DOUBLEBUFFER    , True,
                            None
                    };
            */
            const EGLint attribs[] = {
                    EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                    EGL_BLUE_SIZE, 8,
                    EGL_GREEN_SIZE, 8,
                    EGL_RED_SIZE, 8,
                    EGL_DEPTH_SIZE, 24,
                    EGL_NONE
            };

            EGLint w, h, format;
            EGLint numConfigs;
            EGLConfig config = nullptr;

            //ANativeWindow *window;

            //RAC_ERRNO_MSG("extended_window before glXChooseFBConfig");

            /*
            int fbcount = 0;
            int screen = DefaultScreen(dpy);
            GLXFBConfig* fbc = glXChooseFBConfig(dpy, screen, visual_attribs, &fbcount);
            if (!fbc) {
                ILLIXR::abort("Failed to retrieve a framebuffer config");
            }

            int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;
            int i;
            for (i = 0; i < fbcount; ++i) {
                XVisualInfo *vi = glXGetVisualFromFBConfig(dpy, fbc[i]);
                if (vi) {
                    int samp_buf, samples;
                    glXGetFBConfigAttrib(dpy, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf);
                    glXGetFBConfigAttrib(dpy, fbc[i], GLX_SAMPLES       , &samples );
                    if (best_fbc < 0 || (samp_buf && samples > best_num_samp)) {
                        best_fbc = i, best_num_samp = samples;
                    }
                    if (worst_fbc < 0 || (!samp_buf || samples < worst_num_samp)) {
                        worst_fbc = i, worst_num_samp = samples;
                    }
                }
                XFree(vi);
            }

            assert(0 <= best_fbc && best_fbc < fbcount);
            GLXFBConfig bestFbc = fbc[best_fbc];
            */
            eglChooseConfig(display, attribs, nullptr, 0, &numConfigs);
            LOGI("config chosen");
            std::unique_ptr < EGLConfig[] > supportedConfigs(new EGLConfig[numConfigs]);
            assert(supportedConfigs);
            eglChooseConfig(display, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
            assert(numConfigs);
            LOGI("Num configs %d", numConfigs);
            auto i = 0;
            for (; i < numConfigs; i++) {
                auto &cfg = supportedConfigs[i];
                EGLint r, g, b, d;
                if (eglGetConfigAttrib(display, cfg, EGL_RED_SIZE, &r) &&
                    eglGetConfigAttrib(display, cfg, EGL_GREEN_SIZE, &g) &&
                    eglGetConfigAttrib(display, cfg, EGL_BLUE_SIZE, &b) &&
                    eglGetConfigAttrib(display, cfg, EGL_DEPTH_SIZE, &d) &&
                    r == 8 && g == 8 && b == 8 && d == 24) {

                    config = supportedConfigs[i];
                    break;
                }
            }
            if (i == numConfigs) {
                config = supportedConfigs[0];
            }

            if (config == nullptr) {
                LOGW("Unable to initialize EGLConfig");
                return;
                //return -1;
            }
            LOGI("EGL config initialized successfully");

            eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);
            LOGI("EGL get config attr worked");
            if (window != nullptr) {
                LOGI("window is not nullptr");
            }
            else{
                LOGI("window is nullptr");
            }
            try {
                surface = eglCreateWindowSurface(display, config, window, nullptr);
            }
            catch (const std::exception& e) {
                LOGI("Eroorreewfrefe");
                return;
            }
            LOGI("EGL create window surface wprked");
            context = eglCreateContext(display, config, nullptr, nullptr);


            if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
                LOGW("Unable to eglMakeCurrent");
                return;
                //return -1;
            }
            LOGI("EGL Make current worked");

            eglQuerySurface(display, surface, EGL_WIDTH, &w);
            eglQuerySurface(display, surface, EGL_HEIGHT, &h);

            // Free the FBConfig list allocated by glXChooseFBConfig()
            //XFree(fbc);
/*
            auto opengl_info = {GL_VENDOR, GL_RENDERER, GL_VERSION, GL_EXTENSIONS};
            for (auto name : opengl_info) {
                auto info = glGetString(name);
                LOGI("OpenGL Info: %s", info);
            }
            // Initialize GL state.
            glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
            glEnable(GL_CULL_FACE);
            glShadeModel(GL_SMOOTH);
            glDisable(GL_DEPTH_TEST);

*/
            // Get a visual
            /*
            XVisualInfo *vi = glXGetVisualFromFBConfig(dpy, bestFbc);
            _m_cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

            XSetWindowAttributes attributes;
            attributes.colormap = _m_cmap;
            attributes.background_pixel = 0;
            attributes.border_pixel = 0;
            attributes.event_mask = ExposureMask | KeyPressMask;
            win = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
                                CWBackPixel | CWColormap | CWBorderPixel | CWEventMask, &attributes);
            if (!win) {
                ILLIXR::abort("Failed to create window");
            }
            XStoreName(dpy, win, "ILLIXR Extended Window");
            XMapWindow(dpy, win);

            // Done with visual info
            XFree(vi);

            glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
            glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
                    glXGetProcAddressARB((const GLubyte *) "glXCreateContextAttribsARB");
            int context_attribs[] = {
                    GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
                    GLX_CONTEXT_MINOR_VERSION_ARB, 3,
                    None
            };

            glc = glXCreateContextAttribsARB(dpy, bestFbc, _shared_gl_context, True, context_attribs);

            // Sync to process errors
            RAC_ERRNO_MSG("extended_window before XSync");
            XSync(dpy, false);
            RAC_ERRNO_MSG("extended_window after XSync");
            */
            LOGI("constructor ended");

        }

        ~xlib_gl_extended_window() {
            if (display != EGL_NO_DISPLAY) {
                eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
                if (context != EGL_NO_CONTEXT) {
                    eglDestroyContext(display, context);
                }
                if (surface != EGL_NO_SURFACE) {
                    eglDestroySurface(display, surface);
                }
                eglTerminate(display);
            }
            display = EGL_NO_DISPLAY;
            context = EGL_NO_CONTEXT;
            surface = EGL_NO_SURFACE;
            /*
            RAC_ERRNO_MSG("xlib_gl_extended_window at start of destructor");

            [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(dpy, None, nullptr));
            assert(gl_result && "glXMakeCurrent should not fail");

            glXDestroyContext(dpy, glc);
            XDestroyWindow(dpy, win);
            Window root = DefaultRootWindow(dpy);
            XDestroyWindow(dpy, root);
            XFreeColormap(dpy, _m_cmap);

            /// See [documentation](https://tronche.com/gui/x/xlib/display/XCloseDisplay.html)
            /// See [example](https://www.khronos.org/opengl/wiki/Programming_OpenGL_in_Linux:_GLX_and_Xlib)
            XCloseDisplay(dpy);

            RAC_ERRNO_MSG("xlib_gl_extended_window at end of destructor");
             */
        }

        /* private:
             Colormap _m_cmap;
         };
         */
    };
}


/*
//GLX context magics
#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

namespace ILLIXR {
    class xlib_gl_extended_window : public phonebook::service {
    public:
        int        width;
        int        height;
        Display*   dpy;
        Window     win;
        GLXContext glc;
        xlib_gl_extended_window(int _width, int _height, GLXContext _shared_gl_context) {
            width = _width;
            height = _height;

#ifndef NDEBUG
            std::cout << "Opening display" << std::endl;
#endif
            RAC_ERRNO_MSG("extended_window at start of xlib_gl_extended_window constructor");

            dpy = XOpenDisplay(nullptr);
            if (dpy == nullptr) {
                ILLIXR::abort("Cannot connect to X server");
            } else {
				// Apparently, XOpenDisplay's _true_ error indication is whether dpy is nullptr.
				// https://cboard.cprogramming.com/linux-programming/119957-xlib-perversity.html
				// if (errno != 0) {
				// 	std::cerr << "XOpenDisplay succeeded, but errno = " << errno << "; This is benign, so I'm clearing it now.\n";
				// }
				errno = 0;
            }

            Window root = DefaultRootWindow(dpy);
            // Get a matching FB config
            static int visual_attribs[] =
            {
                GLX_X_RENDERABLE    , True,
                GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
                GLX_RENDER_TYPE     , GLX_RGBA_BIT,
                GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
                GLX_RED_SIZE        , 8,
                GLX_GREEN_SIZE      , 8,
                GLX_BLUE_SIZE       , 8,
                GLX_ALPHA_SIZE      , 8,
                GLX_DEPTH_SIZE      , 24,
                GLX_STENCIL_SIZE    , 8,
                GLX_DOUBLEBUFFER    , True,
                None
            };

#ifndef NDEBUG
            std::cout << "Getting matching framebuffer configs" << std::endl;
#endif
            RAC_ERRNO_MSG("extended_window before glXChooseFBConfig");

            int fbcount = 0;
            int screen = DefaultScreen(dpy);
            GLXFBConfig* fbc = glXChooseFBConfig(dpy, screen, visual_attribs, &fbcount);
            if (!fbc) {
                ILLIXR::abort("Failed to retrieve a framebuffer config");
            }

#ifndef NDEBUG
            std::cout << "Found " << fbcount << " matching FB configs" << std::endl;

            // Pick the FB config/visual with the most samples per pixel
            std::cout << "Getting XVisualInfos" << std::endl;
#endif
            int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;
            int i;
            for (i = 0; i < fbcount; ++i) {
                XVisualInfo *vi = glXGetVisualFromFBConfig(dpy, fbc[i]);
                if (vi) {
                    int samp_buf, samples;
                    glXGetFBConfigAttrib(dpy, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf);
                    glXGetFBConfigAttrib(dpy, fbc[i], GLX_SAMPLES       , &samples );
#ifndef NDEBUG
                    std::cout << "Matching fbconfig " << i
                              << ", visual ID 0x" << std::hex << vi->visualid << std::dec
                              << ": SAMPLE_BUFFERS = " << samp_buf
                              << ", SAMPLES = " << samples
                              << std::endl;
#endif
                    if (best_fbc < 0 || (samp_buf && samples > best_num_samp)) {
                        best_fbc = i, best_num_samp = samples;
                    }
                    if (worst_fbc < 0 || (!samp_buf || samples < worst_num_samp)) {
                        worst_fbc = i, worst_num_samp = samples;
                    }
                }
                XFree(vi);
            }

            assert(0 <= best_fbc && best_fbc < fbcount);
            GLXFBConfig bestFbc = fbc[best_fbc];

            // Free the FBConfig list allocated by glXChooseFBConfig()
            XFree(fbc);

            // Get a visual
            XVisualInfo *vi = glXGetVisualFromFBConfig(dpy, bestFbc);
#ifndef NDEBUG
            std::cout << "Chose visual ID = 0x" << std::hex << vi->visualid << std::dec << std::endl;

            std::cout << "Creating colormap" << std::endl;
#endif
            _m_cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

#ifndef NDEBUG
            std::cout << "Creating window" << std::endl;
#endif
            XSetWindowAttributes attributes;
            attributes.colormap = _m_cmap;
            attributes.background_pixel = 0;
            attributes.border_pixel = 0;
            attributes.event_mask = ExposureMask | KeyPressMask;
            win = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
                CWBackPixel | CWColormap | CWBorderPixel | CWEventMask, &attributes);
            if (!win) {
                ILLIXR::abort("Failed to create window");
            }
            XStoreName(dpy, win, "ILLIXR Extended Window");
            XMapWindow(dpy, win);

            // Done with visual info
            XFree(vi);

#ifndef NDEBUG
            std::cout << "Creating context" << std::endl;
#endif
            glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
            glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
                glXGetProcAddressARB((const GLubyte *) "glXCreateContextAttribsARB");
            int context_attribs[] = {
                GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
                GLX_CONTEXT_MINOR_VERSION_ARB, 3,
                None
            };

            glc = glXCreateContextAttribsARB(dpy, bestFbc, _shared_gl_context, True, context_attribs);

            // Sync to process errors
            RAC_ERRNO_MSG("extended_window before XSync");
            XSync(dpy, false);
            RAC_ERRNO_MSG("extended_window after XSync");

#ifndef NDEBUG
            // Doing glXMakeCurrent here makes a third thread, the runtime one, enter the mix, and
            // then there are three GL threads: runtime, timewarp, and gldemo, and the switching of
            // contexts without synchronization during the initialization phase causes a data race.
            // This is why native.yaml sometimes succeeds and sometimes doesn't. Headless succeeds
            // because this behavior is OpenGL implementation dependent, and apparently mesa
            // differs from NVIDIA in this regard.
            // The proper fix is #173. Comment the below back in once #173 is done. In any case,
            // this is just for debugging and does not affect any functionality.


            const bool gl_result_0 = static_cast<bool>(glXMakeCurrent(dpy, win, glc));
            int major = 0, minor = 0;
            glGetIntegerv(GL_MAJOR_VERSION, &major);
            glGetIntegerv(GL_MINOR_VERSION, &minor);
            std::cout << "OpenGL context created" << std::endl
                      << "Version " << major << "." << minor << std::endl
                      << "Vender " << glGetString(GL_VENDOR) << std::endl
                      << "Renderer " << glGetString(GL_RENDERER) << std::endl;
            const bool gl_result_1 = static_cast<bool>(glXMakeCurrent(dpy, None, nullptr));
            */
/*
#endif
        }

        ~xlib_gl_extended_window() {
            RAC_ERRNO_MSG("xlib_gl_extended_window at start of destructor");

            [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(dpy, None, nullptr));
            assert(gl_result && "glXMakeCurrent should not fail");

            glXDestroyContext(dpy, glc);
            XDestroyWindow(dpy, win);
            Window root = DefaultRootWindow(dpy);
            XDestroyWindow(dpy, root);
            XFreeColormap(dpy, _m_cmap);

            /// See [documentation](https://tronche.com/gui/x/xlib/display/XCloseDisplay.html)
            /// See [example](https://www.khronos.org/opengl/wiki/Programming_OpenGL_in_Linux:_GLX_and_Xlib)
            XCloseDisplay(dpy);

            RAC_ERRNO_MSG("xlib_gl_extended_window at end of destructor");
        }

    private:
        Colormap _m_cmap;
    };
}
*/