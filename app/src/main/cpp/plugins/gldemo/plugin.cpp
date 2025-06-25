#include "plugin.hpp"

#include "illixr/error_util.hpp"
#include "illixr/global_module_defs.hpp"
#include "illixr/shader_util.hpp"
#include "shaders/demo_shader.hpp"
#include "illixr/math_util.hpp"

#include <EGL/egl.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

using namespace ILLIXR;

// Wake up 1 ms after vsync instead of exactly at vsync to account for scheduling uncertainty
static constexpr std::chrono::milliseconds VSYNC_SAFETY_DELAY{1};

const Eigen::Quaternionf INIT = {0.9238795, 0., 0.3826834, 0.};


[[maybe_unused]] gldemo::gldemo(const std::string &name_, phonebook *pb_)
        : threadloop{name_, pb_}, xwin_{pb->lookup_impl<xlib_gl_extended_window>()},
          switchboard_{pb->lookup_impl<switchboard>()},
          log_service_{pb->lookup_impl<log_service>()},
          pose_prediction_{pb->lookup_impl<data_format::pose_prediction>()},
          lock_{pb->lookup_impl<common_lock>()},
          clock_{pb->lookup_impl<RelativeClock>()},
          vsync_{switchboard_->get_reader<switchboard::event_wrapper<time_point>>(
                  "vsync_estimate")},
          image_handle_{switchboard_->get_writer<data_format::image_handle>("image_handle")},
          eyebuffer_{switchboard_->get_writer<ILLIXR::data_format::rendered_frame>("eyebuffer")} {}

// Essentially, a crude equivalent of XRWaitFrame.
void gldemo::wait_vsync() {
    switchboard::ptr<const switchboard::event_wrapper<time_point>> next_vsync = vsync_.get_ro_nullable();
    time_point now = clock_->now();
    time_point wait_time;

    if (next_vsync == nullptr) {
        // If no vsync data available, just sleep for roughly a vsync period.
        // We'll get synced back up later.
        std::this_thread::sleep_for(display_params::period);
        return;
    }

#ifndef NDEBUG
    if (log_count > LOG_PERIOD) {
        double vsync_in = duration2double<std::milli>(**next_vsync - now);
        std::cout << "\033[1;32m[GL DEMO APP]\033[0m First vsync is in " << vsync_in << "ms"
                  << std::endl;
    }
#endif

    bool hasRenderedThisInterval = (now - last_time_) < display_params::period;

    // If less than one frame interval has passed since we last rendered...
    if (hasRenderedThisInterval) {
        // We'll wait until the next vsync, plus a small delay time.
        // Delay time helps with some inaccuracies in scheduling.
        wait_time = **next_vsync + VSYNC_SAFETY_DELAY;

        // If our sleep target is in the past, bump it forward
        // by a vsync period, so it's always in the future.
        while (wait_time < now) {
            wait_time += display_params::period;
        }

#ifndef NDEBUG
        if (log_count > LOG_PERIOD) {
            double wait_in = duration2double<std::milli>(wait_time - now);
            std::cout << "\033[1;32m[GL DEMO APP]\033[0m Waiting until next vsync, in "
                      << wait_in << "ms" << std::endl;
        }
#endif
        // Perform the sleep.
        // TODO: Consider using Monado-style sleeping, where we nanosleep for
        // most of the wait, and then spin-wait for the rest?
        std::this_thread::sleep_for(wait_time - now);
    } else {
#ifndef NDEBUG
        if (log_count > LOG_PERIOD) {
            std::cout
                    << "\033[1;32m[GL DEMO APP]\033[0m We haven't rendered yet, rendering immediately."
                    << std::endl;
        }
#endif
    }
}

void gldemo::_p_thread_setup() {
    last_time_ = clock_->now();

    // Note: glXMakeContextCurrent must be called from the thread which will be using it.

}

void gldemo::_p_one_iteration() {
    auto start = std::chrono::high_resolution_clock::now();
    // Essentially, XRWaitFrame.
    wait_vsync();

    lock_->get_lock();
    [[maybe_unused]] const bool gl_result = static_cast<bool>(eglMakeCurrent(xwin_->display_,
                                                                             xwin_->surface_,
                                                                             xwin_->surface_,
                                                                             xwin_->context_));
    assert(gl_result && "eglMakeCurrent should not fail");

    glUseProgram(demo_shader_program_);
    glBindFramebuffer(GL_FRAMEBUFFER, eye_texture_FBO_);

    glUseProgram(demo_shader_program_);
    glBindVertexArray(demo_vao_);
    glViewport(0, 0, display_params::width_pixels, display_params::height_pixels);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    glClearDepthf(1);

    // We'll calculate this model view matrix
    // using fresh pose data, if we have any.

    Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();

    const data_format::fast_pose_type fast_pose = pose_prediction_->get_fast_pose();
    data_format::pose_type pose = fast_pose.pose;

    Eigen::Matrix3f head_rotation_matrix = pose.orientation.toRotationMatrix();
    ANDROID_LOG("Pose3: %f %f %f %f %f %f %f", pose.position.x(), pose.position.y(),
                pose.position.z(),
                pose.orientation.w(), pose.orientation.x(), pose.orientation.y(),
                pose.orientation.z());
    // Excessive? Maybe.
    constexpr int LEFT_EYE = 0;

    for (auto eye_idx = 0; eye_idx < 2; eye_idx++) {
        // Offset of eyeball from pose
        auto eyeball =
                Eigen::Vector3f((eye_idx == LEFT_EYE ? -display_params::ipd / 2.0f :
                                 display_params::ipd / 2.0f), 1.5, 4.0);

        // Apply head rotation to eyeball offset vector
        eyeball = head_rotation_matrix * eyeball;

        // Apply head position to eyeball
        eyeball += pose.position;

        // Build our eye matrix from the pose's position + orientation.
        Eigen::Matrix4f eye_matrix = Eigen::Matrix4f::Identity();
        eye_matrix.block<3, 1>(0, 3) = eyeball; // Set position to eyeball's position
        Eigen::Quaternionf rot = INIT * pose.orientation;
        eye_matrix.block<3, 3>(0, 0) = rot.toRotationMatrix();

        // Objects' "view matrix" is inverse of eye matrix.
        auto view_matrix = eye_matrix.inverse();

        Eigen::Matrix4f modelViewMatrix = modelMatrix * view_matrix;
        glUniformMatrix4fv(model_view_attr_, 1, GL_FALSE, (GLfloat *) (modelViewMatrix.data()));
        glUniformMatrix4fv(projection_attr_, 1, GL_FALSE, (GLfloat *) (basic_projection_.data()));

        glBindTexture(GL_TEXTURE_2D, eye_textures_[eye_idx]);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                               eye_textures_[eye_idx], 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f);

        RAC_ERRNO_MSG("gldemo before glClear");
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        RAC_ERRNO_MSG("gldemo after glClear");

        demo_scene_.Draw();
    }

    glFinish();

#ifndef NDEBUG
    const double frame_duration_s = duration2double(clock_->now() - last_time_);
    const double fps = 1.0 / frame_duration_s;

    if (log_count > LOG_PERIOD) {
        std::cout << "\033[1;32m[GL DEMO APP]\033[0m Submitting frame to buffer "
                  << which_buffer_
                  << ", frametime: " << frame_duration_s << ", FPS: " << fps << std::endl;
    }
#endif
    last_time_ = clock_->now();

    /// Publish our submitted frame handle to Switchboard!
//        _m_eyebuffer.put(_m_eyebuffer.allocate<rendered_frame>(rendered_frame{
//            // Somehow, C++ won't let me construct this object if I remove the `rendered_frame{` and `}`.
//            // `allocate<rendered_frame>(...)` _should_ forward the arguments to rendered_frame's constructor, but I guess
//            // not.
//            std::array<GLuint, 2>{eyeTextures[0], eyeTextures[1]},
//            std::array<GLuint, 2>{which_buffer, which_buffer},
//            fast_pose,
//            fast_pose.predict_computed_time, lastTime}
//            ));

    eyebuffer_.put(
            eyebuffer_.allocate<data_format::rendered_frame>(data_format::rendered_frame{
                    // Somehow, C++ won't let me construct this object if I remove the `rendered_frame{` and `}`.
                    // `allocate<rendered_frame>(...)` _should_ forward the arguments to rendered_frame's constructor, but I guess
                    // not.
                    std::array<GLuint, 2>{0, 0},
                    std::array<GLuint, 2>{which_buffer_, which_buffer_}, fast_pose,
                    fast_pose.predict_computed_time, last_time_}));

    which_buffer_ = !which_buffer_;


    [[maybe_unused]] const bool gl_result_1 = static_cast<bool>(eglMakeCurrent(xwin_->display_,
                                                                               NULL, NULL,
                                                                               nullptr));
    assert(gl_result_1 && "glXMakeCurrent should not fail");
    lock_->release_lock();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    ANDROID_LOG("duration: %f", duration2double(duration));
    log_service_->write_duration("gldemo", duration2double(duration));
#ifndef NDEBUG
    if (log_count > LOG_PERIOD) {
        log_count = 0;
    } else {
        log_count++;
    }
#endif
}

void gldemo::createSharedEyebuffer(GLuint *texture_handle) {
    // Create the shared eye texture handle
    glGenTextures(1, texture_handle);
    glBindTexture(GL_TEXTURE_2D, *texture_handle);

    // Set the texture parameters for the texture that the FBO will be mapped into
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, display_params::width_pixels,
                 display_params::height_pixels, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, 0);

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
}

void gldemo::createFBO(GLuint *texture_handle, GLuint *fbo, GLuint *depth_target) {
    // Create a framebuffer to draw some things to the eye texture
    glGenFramebuffers(1, fbo);

    // Bind the FBO as the active framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, *fbo);
    glGenRenderbuffers(1, depth_target);
    glBindRenderbuffer(GL_RENDERBUFFER, *depth_target);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, display_params::width_pixels,
                          display_params::height_pixels);
    // glRenderbufferStorageMultisample(GL_RENDERBUFFER, fboSampleCount, GL_DEPTH_COMPONENT, display_params::width_pixels,
    // display_params::height_pixels);

    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    // Bind eyebuffer texture
    std::cout << "About to bind eyebuffer texture, texture handle: " << *texture_handle
              << std::endl;

    glBindTexture(GL_TEXTURE_2D, *texture_handle);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           *texture_handle, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    // attach a renderbuffer to depth attachment point
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                              *depth_target);

    // Unbind FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void gldemo::start() {
    lock_->get_lock();
    [[maybe_unused]] const bool gl_result_0 = static_cast<bool>(eglMakeCurrent(xwin_->display_,
                                                                               xwin_->surface_,
                                                                               xwin_->surface_,
                                                                               xwin_->context_));
    assert(gl_result_0 && "glXMakeCurrent should not fail");

    // Init and verify GLEW
    /*
    const GLenum glew_err = glewInit();
    if (glew_err != GLEW_OK) {
        std::cerr << "[gldemo] GLEW Error: " << glewGetErrorString(glew_err) << std::endl;
        ILLIXR::abort("[gldemo] Failed to initialize GLEW");
    }
    */

    //glEnable(GL_DEBUG_OUTPUT);
    //glDebugMessageCallback(MessageCallback, 0);

    // Create two shared eye textures, one for each eye
    createSharedEyebuffer(&(eye_textures_[0]));
    image_handle_.put(image_handle_.allocate<data_format::image_handle>(
            data_format::image_handle{eye_textures_[0], 1,
                                      data_format::swapchain_usage::LEFT_SWAPCHAIN}));
    createSharedEyebuffer(&(eye_textures_[1]));
    image_handle_.put(image_handle_.allocate<data_format::image_handle>(
            data_format::image_handle{eye_textures_[1], 1,
                                      data_format::swapchain_usage::RIGHT_SWAPCHAIN}));
    // Initialize FBO and depth targets, attaching to the frame handle
    createFBO(&(eye_textures_[0]), &eye_texture_FBO_, &eye_texture_depth_target_);

    // Create and bind global VAO object
    glGenVertexArrays(1, &demo_vao_);
    glBindVertexArray(demo_vao_);

    demo_shader_program_ = init_and_link(demo_vertex_shader, demo_fragment_shader);
#ifndef NDEBUG
    std::cout << "Demo app shader program is program " << demo_shader_program_ << std::endl;
#endif

    //vertexPosAttr = glGetAttribLocation(demoShaderProgram, "vertexPosition");
    //vertexNormalAttr = glGetAttribLocation(demoShaderProgram, "vertexNormal");
    model_view_attr_ = glGetUniformLocation(demo_shader_program_, "u_modelview");
    projection_attr_ = glGetUniformLocation(demo_shader_program_, "u_projection");
    //colorUniform = glGetUniformLocation(demoShaderProgram, "u_color");

    // Load/initialize the demo scene
    char *obj_dir = std::getenv("ILLIXR_DEMO_DATA");
    if (obj_dir == nullptr) {
        ILLIXR::abort("Please define ILLIXR_DEMO_DATA.");
    }

    demo_scene_ = ObjScene(std::string(obj_dir), "scene.obj");

    // Construct perspective projection matrix
    ILLIXR::math_util::projection_fov(&basic_projection_, display_params::fov_x / 2.0f,
                                      display_params::fov_x / 2.0f,
                                      display_params::fov_y / 2.0f,
                                      display_params::fov_y / 2.0f, rendering_params::near_z,
                                      rendering_params::far_z);

    [[maybe_unused]] const bool gl_result_1 = static_cast<bool>(eglMakeCurrent(xwin_->display_,
                                                                               NULL, NULL,
                                                                               nullptr));
    assert(gl_result_1 && "glXMakeCurrent should not fail");
    lock_->release_lock();

    // Effectively, last vsync was at zero.
    // Try to run gldemo right away.
    threadloop::start();
}


PLUGIN_MAIN(gldemo)
