#pragma once

#include "illixr/data_format/frame.hpp"
#include "illixr/data_format/misc.hpp"
#include "illixr/data_format/pose_prediction.hpp"
#include "illixr/common_lock.hpp"
#include "illixr/extended_window.hpp"
#include "illixr/gl_util/obj.hpp"
#include "illixr/log_service.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"
#include <android/log.h>

#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "gldemo", __VA_ARGS__))

namespace ILLIXR {

class gldemo : public threadloop {
public:
    // Public constructor, create_component passes Switchboard handles ("plugs")
    // to this constructor. In turn, the constructor fills in the private
    // references to the switchboard plugs, so the component can read the
    // data whenever it needs to.
    [[maybe_unused]] gldemo(const std::string &name_, phonebook *pb_);

    // Essentially, a crude equivalent of XRWaitFrame.
    void wait_vsync();

    void _p_thread_setup() override;

    void _p_one_iteration() override;

    // We override start() to control our own lifecycle
    void start() override;

#ifndef NDEBUG
    size_t log_count = 0;
    size_t LOG_PERIOD = 20;
#endif

private:
    const std::shared_ptr<const xlib_gl_extended_window> xwin_;
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<log_service> log_service_;
    const std::shared_ptr<data_format::pose_prediction> pose_prediction_;
    const std::shared_ptr<common_lock> lock_;
    const std::shared_ptr<const RelativeClock> clock_;
    const switchboard::reader<switchboard::event_wrapper<time_point>> vsync_;

    // Switchboard plug for application eye buffer.
    // We're not "writing" the actual buffer data,
    // we're just atomically writing the handle to the
    // correct eye/framebuffer in the "swapchain".
    switchboard::writer<data_format::image_handle> image_handle_;
    switchboard::writer<data_format::rendered_frame> eyebuffer_;

    GLuint eye_textures_[2];
    GLuint eye_texture_FBO_;
    GLuint eye_texture_depth_target_;

    unsigned char which_buffer_ = 0;

    GLuint demo_vao_;
    GLuint demo_shader_program_;

    //GLuint vertexPosAttr;
    //GLuint vertexNormalAttr;
    GLuint model_view_attr_;
    GLuint projection_attr_;

    //GLuint colorUniform;

    ObjScene demo_scene_;

    Eigen::Matrix4f basic_projection_;

    time_point last_time_;

    void createSharedEyebuffer(GLuint *texture_handle);

    void createFBO(GLuint *texture_handle, GLuint *fbo, GLuint *depth_target);

};

}