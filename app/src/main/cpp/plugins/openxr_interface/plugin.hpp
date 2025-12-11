#pragma once
#ifndef XR_USE_GRAPHICS_API_OPENGL_ES
#define XR_USE_GRAPHICS_API_OPENGL_ES
#endif
#ifndef XR_USE_PLATFORM_ANDROID
#define XR_USE_PLATFORM_ANDROID
#endif

#include "stereo_renderer.hpp"

#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"
#include "illixr/data_format/pose.hpp"
#include "illixr/data_format/frame.hpp"

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include "frame_dumper.hpp"

#define OXR_CheckErrors(cmd, pfunc) do { \
    XrResult res = cmd; \
    if (XR_FAILED(res)) { \
        spdlog::get("illixr")->error("OpenXR error {} at {}:{} from {}", static_cast<int>(res), __FILE__, __LINE__, #pfunc); \
        throw std::runtime_error("Call failed");                                     \
    } \
} while(0)
#define DECL_PFN(pfn) PFN_##pfn pfn = nullptr
#define OXR(func) OXR_CheckErrors(func, #func);
#define INIT_PFN(pfn, pf) OXR(xrGetInstanceProcAddr(instance_, #pfn, (PFN_xrVoidFunction*)(&pf)))

namespace ILLIXR {
class oxr_interface : public threadloop {
public:
    [[maybe_unused]] oxr_interface(const std::string& name_, phonebook* pb_);
    ~oxr_interface() override;

    std::vector<XrApiLayerProperties>& get_layer_properties() {
        return layer_properties_;
    }

    void _p_thread_setup() override;
    XrSession session() const {
        return session_;
    }

    XrInstance instance() const {
        return instance_;
    }

protected:
    void _p_one_iteration() override;
private:
    void create_swapchains();
    bool ensure_gl_context();
    void get_openxr_projection_matrix(int eye, float* mvp);
    void bind_eye_framebuffer(int eye);
    void release_swapchain_image(int eye);
    void create_projection_fov(float* result, XrFovf fov, float near_z, float far_z);
    void create_view_matrix(float* result, XrPosef pose);
    void identity_matrix(float* result);
    void matrix_multiply(float* result, const float* a, const float* b);

    void init_xr();
    void create_session();
    void poll_events();
    void run_frame();

    XrResult get_head_pose(XrTime time, XrPosef* outPose, XrBool32* outValid);


    const std::shared_ptr<switchboard> switchboard_;
    struct android_app* app_;
    const std::shared_ptr<relative_clock> clock_;
    switchboard::writer<data_format::pose_type> pose_writer_;
    switchboard::buffered_reader<data_format::dual_frames> frame_reader_;

    std::shared_ptr<const data_format::dual_frames> current_frames_ = nullptr;

    XrSession session_ = XR_NULL_HANDLE;
    XrInstance instance_ = XR_NULL_HANDLE;
    std::vector<XrApiLayerProperties> layer_properties_;
    std::vector<const char*> required_extensions_;
    std::vector<XrExtensionProperties> extension_properties_;

    std::unique_ptr<stereo_renderer> renderer_;

    //std::unique_ptr<frame_dumper> dumper_[2];
    uint64_t frame_counter_{0};
};

}