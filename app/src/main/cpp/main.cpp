#include "extended_window.hpp"
#include <android_native_app_glue.h>
#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/log.h>
#include <thread>
#include <vector>
#include "runtime/main.h"

using namespace ILLIXR;

int x = 0;
static void engine_draw_frame(xlib_gl_extended_window* xwin) {
    if (xwin->display == nullptr) {
        // No display
        LOGI("OpenGL Info:");
        return;
    }
    x++;
    // Just fill the screen with a color.
    glClearColor(x%3, x%7,
                 x%5, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    eglSwapBuffers(xwin->display, xwin->surface);
    LOGI("DONE DRAWING FRAME");
}
static void handle_cmd(struct android_app* app, int32_t cmd) {
    LOGI("Got command %d", cmd);
    switch(cmd) {
        case APP_CMD_INIT_WINDOW:
        {
            EGLContext context = EGL_NO_CONTEXT;
            xlib_gl_extended_window* xwin = new xlib_gl_extended_window{1, 1, context, app->window};
            int i = 10000000;

            while(i>0) {
                engine_draw_frame(xwin);
                i--;
            }
            break;
        }
        default:
            LOGI("Some other command");
    }
}


void  android_main(struct android_app* state) {

    state->onAppCmd = handle_cmd;
    LOGI("ACTIVITY STATE %d", state->activityState);
    //const std::shared_ptr<xlib_gl_extended_window> xwin;
    //EGLContext context = EGL_NO_CONTEXT;
    LOGI("OpenGL Info: before initialization");
   /* while(state->window == nullptr) {
        //LOGI("Still null");
        ;
    }*/

    std::vector<std::string> arguments = {"offline_imu_cam/plugin.opt.so", "gtsam_integrator/plugin.opt.so", "pose_prediction/plugin.opt.so", "ground_truth_slam/plugin.opt.so", "gldemo/plugin.opt.so", "debugview/plugin.opt.so", "offload_data/plugin.opt.so", "timewarp_gl/plugin.opt.so"};
    std::vector<char*> argv;
    for (const auto& arg : arguments)
        argv.push_back((char*)arg.data());
    argv.push_back(nullptr);

    //char *arg = "mm";
    //char *argv[1] = {arg};
    //const int argc = 1;
    //runtime_main(argv.size() - 1, argv.data());
    //std::thread runtime_thread(runtime_main, argv.size() - 1, argv.data());
    LOGI("Not null");
    /*
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
     */
    //xlib_gl_extended_window* xwin = new xlib_gl_extended_window{1, 1, context, state->window};
    //eglSwapInterval(xwin->display, 1);
   /*
    int i = 100000000;

    while(true) {
        engine_draw_frame(xwin);
        i--;
        break;
    }
    */

    //eglSwapBuffers(xwin->display, xwin->surface);
}