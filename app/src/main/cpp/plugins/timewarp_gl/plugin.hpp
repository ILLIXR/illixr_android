#pragma once

#include "illixr/data_format/frame.hpp"
#include "illixr/data_format/misc.hpp"
#include "illixr/data_format/pose_prediction.hpp"
#include "illixr/common_lock.hpp"
#include "illixr/hmd.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

#define EGL_EGLEXT_PROTOTYPES 1
#define GL_GLEXT_PROTOTYPES
//#define ILLIXR_MONADO 1

#include <EGL/egl.h>

namespace ILLIXR {

class timewarp_gl : public threadloop {
public:
    // Public constructor, create_component passes Switchboard handles ("plugs")
    // to this constructor. In turn, the constructor fills in the private
    // references to the switchboard plugs, so the component can read the
    // data whenever it needs to.
    timewarp_gl(const std::string& name_, phonebook* pb_);

    skip_option _p_should_skip() override;

    void _p_thread_setup() override;

    void _p_one_iteration() override;

#ifndef NDEBUG
    size_t log_count = 0;
    size_t LOG_PERIOD = 20;
#endif

private:
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<data_format::pose_prediction> pose_prediction_;
    const std::shared_ptr<common_lock> lock_;
    const std::shared_ptr<const RelativeClock> clock_;
    // OpenGL objects
    EGLDisplay display_;
#ifndef ILLIXR_MONADO
    ANativeWindow* window_;
#endif
    EGLSurface surface_;
    EGLContext glcontext_;
    // Note: 0.9 works fine without hologram, but we need a larger safety net with hologram enabled
    static constexpr double DELAY_FRACTION = 0.9;
    // Shared objects between ILLIXR and the application (either gldemo or Monado)
    bool rendering_ready_;
    data_format::graphics_api client_backend_;
    std::atomic<bool> image_handles_ready_;
    std::atomic<bool> semaphore_handles_ready_;

    // Left and right eye images
    std::array<std::vector<data_format::image_handle>, 2> eye_image_handles_;
    std::array<std::vector<GLuint>, 2> eye_swapchains_;
    std::array<size_t, 2> eye_swapchains_size_;

    // Intermediate timewarp framebuffers for left and right eye textures
    std::array<GLuint, 2> eye_output_textures_;
    std::array<GLuint, 2> eye_framebuffers_;

#ifdef ILLIXR_MONADO
    // Semaphores to synchronize between Monado and ILLIXR
        std::array<image_handle, 2>              eye_output_handles_;
        std::array<semaphore_handle, 2>          semaphore_handles_;
        //std::array<GLuint, 2>                  semaphores_;
#endif
    // Switchboard plug for application eye buffer.
    switchboard::reader<data_format::rendered_frame> eyebuffer_;
    switchboard::writer<data_format::illixr_signal> illixr_signal_;

    // Switchboard plug for sending hologram calls
    switchboard::writer<data_format::hologram_input> hologram_;

    // Switchboard plug for publishing vsync estimates
    switchboard::writer<switchboard::event_wrapper<time_point>> vsync_estimate_;

    // Switchboard plug for publishing offloaded data
    switchboard::writer<data_format::texture_pose> offload_data_;

    record_coalescer timewarp_gpu_logger_;
    record_coalescer mtp_logger_;

    GLuint timewarp_shader_program_;

    time_point time_last_swap_;

    HMD::hmd_info_t hmd_info_;

    // Eye sampler array
    GLuint eye_sampler_0_;
    GLuint eye_sampler_1_;

    // Eye index uniform
    GLuint tw_eye_index_unif_;

    // VAOs
    GLuint tw_vao_;

    // Position and UV attribute locations
    GLuint distortion_pos_attr_;
    GLuint distortion_uv0_attr_;
    GLuint distortion_uv1_attr_;
    GLuint distortion_uv2_attr_;

    // Distortion mesh information
    GLuint num_distortion_vertices_;
    GLuint num_distortion_indices_;

    // Distortion mesh CPU buffers and GPU VBO handles
    std::vector<HMD::mesh_coord3d_t> distortion_positions_;
    GLuint distortion_positions_vbo_;
    std::vector<GLuint> distortion_indices_;
    GLuint distortion_indices_vbo_;
    std::vector<HMD::uv_coord_t> distortion_uv0_;
    GLuint distortion_uv0_vbo_;
    std::vector<HMD::uv_coord_t> distortion_uv1_;
    GLuint distortion_uv1_vbo_;
    std::vector<HMD::uv_coord_t> distortion_uv2_;
    GLuint distortion_uv2_vbo_;

    // Handles to the start and end timewarp
    // transform matrices (3x4 uniforms)
    GLuint tw_start_transform_unif_;
    GLuint tw_end_transform_unif_;
    // Basic perspective projection matrix
    GLuint flip_y_unif_;
    Eigen::Matrix4f basic_projection_;

    // Hologram call data
    ullong hologram_seq_{0};
    int prev_counter_ = 0;

    bool disable_warp_;

    bool enable_offload_;

    // PBO buffer for reading texture image
    GLuint PBO_buffer_;

    duration offload_duration_;

    GLubyte* read_texture_image();

    GLuint convert_vk_format_to_gl(int64_t vk_format, GLint swizzle_mask[]);

//    void vulkanGL_interop(const vk_image_handle& vk_handle, int swapchain_index);

//    void import_vulkan_image(const vk_image_handle& vk_handle, swapchain_usage usage);

    void vulkanGL_interop_buffer(const data_format::vk_buffer_handle& vk_buffer_handle,
                                 data_format::swapchain_usage usage);

#ifdef ILLIXR_MONADO
    void import_vulkan_semaphore(const semaphore_handle& vk_handle);
#endif

    void build_timewarp(HMD::hmd_info_t& hmdInfo);

    /* Calculate timewarm transform from projection matrix, view matrix, etc */
    void calculate_timeWarp_transform(Eigen::Matrix4f& transform,
                                      const Eigen::Matrix4f& renderProjectionMatrix,
                                      const Eigen::Matrix4f& renderViewMatrix,
                                      const Eigen::Matrix4f& newViewMatrix);

    // Get the estimated time of the next swap/next Vsync.
    // This is an estimate, used to wait until *just* before vsync.
    time_point get_next_swap_time_estimate();

    // Get the estimated amount of time to put the CPU thread to sleep,
    // given a specified percentage of the total Vsync period to delay.
    duration estimate_time_to_sleep(double framePercentage);
};

}