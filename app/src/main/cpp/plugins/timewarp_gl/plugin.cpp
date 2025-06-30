#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include "plugin.hpp"

#include "illixr/extended_window.hpp"
#include "illixr/global_module_defs.hpp"
#include "illixr/math_util.hpp"
#include "illixr/shader_util.hpp"
#include "shaders/basic_shader.hpp"
#include "shaders/timewarp_shader.hpp"


#include <EGL/eglext.h>

//#define GL_GLEXT_PROTOTYPES
//#include <GLES2/gl2.h>
#include <android/hardware_buffer.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <vulkan/vulkan.h>

using namespace ILLIXR;

//typedef void (*glXSwapIntervalEXTProc)(EGLDisplay *dpy, EGLSurface drawable, int interval);

const record_header timewarp_gpu_record{"timewarp_gpu",
                                        {
                                                {"iteration_no", typeid(std::size_t)},
                                                {"wall_time_start", typeid(time_point)},
                                                {"wall_time_stop", typeid(time_point)},
                                                {"gpu_time_duration",
                                                 typeid(std::chrono::nanoseconds)},
                                        }};

const record_header mtp_record{"mtp_record",
                               {
                                       {"iteration_no", typeid(std::size_t)},
                                       {"vsync", typeid(time_point)},
                                       {"imu_to_display", typeid(std::chrono::nanoseconds)},
                                       {"predict_to_display", typeid(std::chrono::nanoseconds)},
                                       {"render_to_display", typeid(std::chrono::nanoseconds)},
                               }};

timewarp_gl::timewarp_gl(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , switchboard_{pb_->lookup_impl<switchboard>()}
        , pose_prediction_{pb_->lookup_impl<data_format::pose_prediction>()}
        , lock_{pb_->lookup_impl<common_lock>()}
        , clock_{pb_->lookup_impl<relative_clock>()}
        , eyebuffer_{switchboard_->get_reader<ILLIXR::data_format::rendered_frame>("eyebuffer")}
        , illixr_signal_{switchboard_->get_writer<data_format::illixr_signal>("illixr_signal")}
        , hologram_{switchboard_->get_writer<data_format::hologram_input>("hologram_in")}
        , vsync_estimate_{switchboard_->get_writer<switchboard::event_wrapper<time_point>>("vsync_estimate")}
        , offload_data_{switchboard_->get_writer<data_format::texture_pose>("texture_pose")}
        , timewarp_gpu_logger_{record_logger_}, mtp_logger_{record_logger_}
        // TODO: Use #198 to configure this. Delete getenv_or.
        // This is useful for experiments which seek to evaluate the end-effect of timewarp vs no-timewarp.
        // Timewarp poses a "second channel" by which pose data can correct the video stream,
        // which results in a "multipath" between the pose and the video stream.
        // In production systems, this is certainly a good thing, but it makes the system harder to analyze.
        , disable_warp_{switchboard_->get_env_bool("ILLIXR_TIMEWARP_DISABLE", "False")}
        , enable_offload_{switchboard_->get_env_bool("ILLIXR_OFFLOAD_ENABLE", "False")} {
#ifndef ILLIXR_MONADO
    const std::shared_ptr<xlib_gl_extended_window> xwin = pb_->lookup_impl<xlib_gl_extended_window>();
    display_ = xwin->display_;
    window_ = xwin->my_window_;
    surface_ = xwin->surface_;
    glcontext_ = xwin->context_;
    spdlog::get("illixr")->debug("NOT ILLIXR_MONADO ..");
#else
    spdlog::get("illixr")->debug("ILLIXR_MONADO ..");
    // If we use Monado, timewarp_gl must create its own GL context because the extended window isn't used
    dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major_version, minor_version;
    eglInitialize(dpy, &major_version, &minor_version);
    const EGLint attribs[] = {
            //EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_RENDERABLE_TYPE,EGL_OPENGL_ES2_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_ALPHA_SIZE, 0,
            // EGL_DEPTH_SIZE, 24,
            EGL_NONE
    };

    //EGLint w, h, format;
    EGLint numConfigs;
    EGLConfig config = nullptr;
    eglChooseConfig(dpy, attribs, &config, 1, &numConfigs);
    std::unique_ptr < EGLConfig[] > supportedConfigs(new EGLConfig[numConfigs]);
    assert(supportedConfigs);
    eglChooseConfig(dpy, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
    assert(numConfigs);
    auto i = 0;
    for (; i < numConfigs; i++) {
        auto &cfg = supportedConfigs[i];
        EGLint r, g, b, d;
        if (eglGetConfigAttrib(dpy, cfg, EGL_RED_SIZE, &r) &&
            eglGetConfigAttrib(dpy, cfg, EGL_GREEN_SIZE, &g) &&
            eglGetConfigAttrib(dpy, cfg, EGL_BLUE_SIZE, &b) &&
            eglGetConfigAttrib(dpy, cfg, EGL_DEPTH_SIZE, &d) &&
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
    EGLint ctxattrb[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
    };
    glc = eglCreateContext(dpy, config, EGL_NO_CONTEXT, ctxattrb);
    surface = EGL_NO_SURFACE;
//                std::cout << "Timewarp creating GL Context" << std::endl;
//                GLint        attr[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None};
//                XVisualInfo* vi;
//                        if (!(dpy = XOpenDisplay(NULL))) {
//                        fprintf(stderr, "cannot connect to X server\n\n");
//                        exit(1);
//                    }
//
//                        /* get root window */
//                                root = DefaultRootWindow(dpy);
//
//                        /* get visual matching attr */
//                                if (!(vi = glXChooseVisual(dpy, 0, attr))) {
//                        fprintf(stderr, "no appropriate visual found\n\n");
//                        exit(1);
//                    }
//
//                        /* create a context using the root window */
//                                if (!(glc = glXCreateContext(dpy, vi, NULL, GL_TRUE))) {
//                        fprintf(stderr, "failed to create context\n\n");
//                        exit(1);
//                    }
#endif
    client_backend_ = data_format::graphics_api::TBD;
    rendering_ready_ = false;
    image_handles_ready_ = false;
//        swapchain_ready     = false;
#ifdef ILLIXR_MONADO
    semaphore_handles_ready    = false;
#else
    semaphore_handles_ready_ = true;
#endif
    switchboard_->schedule<data_format::image_handle>(id_, "image_handle",
                                                      [this](switchboard::ptr<const data_format::image_handle> handle,
                                                             std::size_t) {
                                                          // only 2 swapchains (for the left and right eye) are supported for now.
//            if (handle->swapchain_index == 0) {
//                LOGT("SWAPCHAIN 0 READY");
//            } else if (handle->swapchain_index == 1) {
//                LOGT("SWAPCHAIN 1 READY");
//            } else {
//                return;
//            }
                                                          spdlog::get("illixr")->debug("SCHEDULE STARTED");
#ifdef ILLIXR_MONADO
                                                          static bool left_output_ready = false, right_output_ready = false;
#else
                                                          static bool left_output_ready = true, right_output_ready = true;
#endif
                                                          spdlog::get("illixr")->debug("handle.. %d", handle->usage);
                                                          switch (handle->usage) {
                                                              case data_format::swapchain_usage::LEFT_SWAPCHAIN: {
                                                                  this->eye_image_handles_[0].push_back(
                                                                          *handle);
                                                                  this->eye_swapchains_size_[0] = handle->num_images;
                                                                  break;
                                                              }
                                                              case data_format::swapchain_usage::RIGHT_SWAPCHAIN: {
                                                                  this->eye_image_handles_[1].push_back(
                                                                          *handle);
                                                                  this->eye_swapchains_size_[1] = handle->num_images;
                                                                  break;
                                                              }
#ifdef ILLIXR_MONADO
                                                                  case swapchain_usage::LEFT_RENDER: {
                                                                      this->eye_output_handles_[0] = *handle;
                                                                      left_output_ready = true;
                                                                      break;
                                                                  }
                                                                  case swapchain_usage::RIGHT_RENDER: {
                                                                      this->eye_output_handles_[1] = *handle;
                                                                      right_output_ready = true;
                                                                      break;
                                                                  }
#endif

                                                              default: {
                                                                  std::cout
                                                                          << "Invalid swapchain usage provided"
                                                                          << std::endl;
                                                                  break;
                                                              }
                                                          }
                                                          spdlog::get("illixr")->debug("SWITCH DONE");
//            assert(handle->swapchain_index == 0 || handle->swapchain_index == 1);
//            this->_m_image_handles[handle->swapchain_index].push_back(*handle);
                                                          if (client_backend_ ==
                                                              data_format::graphics_api::TBD) {
                                                              client_backend_ = handle->type;
                                                          } else {
                                                              assert(client_backend_ ==
                                                                     handle->type);
                                                          }
                                                          if (this->eye_image_handles_[0].size() ==
                                                              this->eye_swapchains_size_[0])
                                                              spdlog::get("illixr")->debug("condition 1");
                                                          if (this->eye_image_handles_[1].size() ==
                                                              this->eye_swapchains_size_[1])
                                                              spdlog::get("illixr")->debug("confition 2");
                                                          if (left_output_ready)
                                                              spdlog::get("illixr")->debug("CONDITION 3");
                                                          if (right_output_ready)
                                                              spdlog::get("illixr")->debug("condition 4");
                                                          if (this->eye_image_handles_[0].size() ==
                                                              this->eye_swapchains_size_[0] &&
                                                              this->eye_image_handles_[1].size() ==
                                                              this->eye_swapchains_size_[1] &&
                                                              left_output_ready &&
                                                              right_output_ready) {
                                                              this->image_handles_ready_ = true;
                                                              //std::cout << "IMAGES READY" << std::endl;
                                                              spdlog::get("illixr")->debug("IMAGES READY");
                                                          }
                                                          spdlog::get("illixr")->debug("NOT READY");
                                                      });

#ifdef ILLIXR_MONADO
    //        switchboard_->schedule<semaphore_handle>(id, "semaphore_handle", [this](switchboard::ptr<const semaphore_handle> handle, std::size_t) {
    //                // We need one semaphore to indicate when the reprojection is ready, and another when it's done
    //            static bool left_semaphore_ready = false, right_semaphore_ready = false;
    //            switch (handle->usage) {
    //                case semaphore_usage::LEFT_RENDER_COMPLETE: {
    //                        semaphore_handles_[0] = *handle;
    //                        left_semaphore_ready = true;
    //                        break;
    //                    }
    //                case semaphore_usage::RIGHT_RENDER_COMPLETE: {
    //                        semaphore_handles_[1] = *handle;
    //                        right_semaphore_ready = true;
    //                        break;
    //                    }
    //                default: {
    //                        std::cout << "Invalid semaphore usage provided" << std::endl;
    //                        break;
    //                    }
    //                }
    //
    //                if (left_semaphore_ready && right_semaphore_ready) {
    //                this->semaphore_handles_ready = true;
    //                        LOGT("SEMAPHORE READY");
    //                }
    //            });
#endif
}

GLubyte* timewarp_gl::read_texture_image() {
    const unsigned memSize = display_params::width_pixels * display_params::height_pixels * 3;
    GLubyte* pixels = new GLubyte[memSize];

    // Start timer
    time_point startGetTexTime = clock_->now();

    // Enable PBO buffer
    glBindBuffer(GL_PIXEL_PACK_BUFFER, PBO_buffer_);

    // Read texture image to PBO buffer
    //glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*) 0);

    // Transfer texture image from GPU to Pinned Memory(CPU)
    GLubyte* ptr = NULL;
    //(GLubyte*) glMapNamedBuffer(PBO_buffer, GL_READ_ONLY, sizeof(PBO_buffer), GL_MAP_WRITE_BIT);

    // Copy texture to CPU memory
    memcpy(pixels, ptr, memSize);

    // Unmap the buffer
    //glUnmapBuffer(GL_PIXEL_PACK_BUFFER);

    // Unbind the buffer
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    // Record the image collection time
    offload_duration_ = clock_->now() - startGetTexTime;

#ifndef NDEBUG
    double time = duration_to_double<std::milli>(offload_duration_);
    std::cout << "Texture image collecting time: " << time << "ms" << std::endl;
#endif

    return pixels;
}

GLuint timewarp_gl::convert_vk_format_to_gl(int64_t vk_format, GLint swizzle_mask[]) {
    switch (vk_format) {
        case VK_FORMAT_R8G8B8A8_UNORM:
            return GL_RGBA8;
        case VK_FORMAT_B8G8R8A8_SRGB: {
            swizzle_mask[0] = GL_BLUE;
            swizzle_mask[2] = GL_RED;
        }
        case VK_FORMAT_R8G8B8A8_SRGB:
            return GL_SRGB8_ALPHA8;
        default:
            return 0;
    }
}

/*
void timewarp_gl::vulkanGL_interop(const vk_image_handle &vk_handle, int swapchain_index) {
    [[maybe_unused]] const bool gl_result = static_cast<bool>(glXMakeCurrent(xwin->dpy, xwin->win,
                                                                             xwin->glc));
    assert(gl_result && "glXMakeCurrent should not fail");
    assert(GLEW_EXT_memory_object_fd &&
           "[timewarp_gl] Missing object memory extensions for Vulkan-GL interop");

    // first get the memory handle of the vulkan object
    GLuint memory_handle;
    GLint dedicated = GL_TRUE;
    glCreateMemoryObjectsEXT(1, &memory_handle);
    assert(glIsMemoryObjectEXT(memory_handle) && "GL memory handle must be created correctly");
    glMemoryObjectParameterivEXT(memory_handle, GL_DEDICATED_MEMORY_OBJECT_EXT, &dedicated);
    glImportMemoryFdEXT(memory_handle, vk_handle.allocation_size, GL_HANDLE_TYPE_OPAQUE_FD_EXT,
                        vk_handle.file_descriptor);

    // then use the imported memory as the opengl texture
    GLint swizzle_mask[4] = {GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA};
    GLuint format = ConvertVkFormatToGL(vk_handle.format, swizzle_mask);
    assert(format != 0 && "Given VK format not handled!");
    GLuint image_handle;
    glGenTextures(1, &image_handle);
    glBindTexture(GL_TEXTURE_2D, image_handle);
    glTextureStorageMem2DEXT(image_handle, 1, format, vk_handle.width, vk_handle.height,
                             memory_handle, 0);
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle_mask);
    _m_swapchain[swapchain_index].push_back(image_handle);
}*/

/*
void timewarp_gl::import_vulkan_image(const vk_image_handle &vk_handle, swapchain_usage usage) {
    PFNGLMEMORYOBJECTPARAMETERIVEXTPROC glMemoryObjectParameterivEXT;
    glMemoryObjectParameterivEXT = (PFNGLMEMORYOBJECTPARAMETERIVEXTPROC) eglGetProcAddress(
            "glMemoryObjectParameterivEXT");

    PFNGLCREATEMEMORYOBJECTSEXTPROC glCreateMemoryObjectsEXT;
    glCreateMemoryObjectsEXT = (PFNGLCREATEMEMORYOBJECTSEXTPROC) eglGetProcAddress(
            "glCreateMemoryObjectsEXT");

    PFNGLIMPORTMEMORYFDEXTPROC glImportMemoryFdEXT;
    glImportMemoryFdEXT = (PFNGLIMPORTMEMORYFDEXTPROC) eglGetProcAddress("glImportMemoryFdEXT");

    PFNGLTEXTURESTORAGEMEM2DEXTPROC glTextureStorageMem2DEXT;
    glTextureStorageMem2DEXT = (PFNGLTEXTURESTORAGEMEM2DEXTPROC) eglGetProcAddress(
            "glTextureStorageMem2DEXT");

    [[maybe_unused]] const bool gl_result = static_cast<bool>(eglMakeCurrent(dpy, surface, surface,
                                                                             glc));
    assert(gl_result && "glXMakeCurrent should not fail");
    //assert(GLEW_EXT_memory_object_fd && "[timewarp_gl] Missing object memory extensions for Vulkan-GL interop");

    // first get the memory handle of the vulkan object
    GLuint memory_handle;
    GLint dedicated = GL_TRUE;
    glCreateMemoryObjectsEXT(1, &memory_handle);
    glMemoryObjectParameterivEXT(memory_handle, GL_DEDICATED_MEMORY_OBJECT_EXT, &dedicated);
    glImportMemoryFdEXT(memory_handle, vk_handle.allocation_size, GL_HANDLE_TYPE_OPAQUE_FD_EXT,
                        vk_handle.file_descriptor);

    // then use the imported memory as the opengl texture
    GLint swizzle_mask[4] = {GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA};
    GLuint format = ConvertVkFormatToGL(vk_handle.format, swizzle_mask);
    assert(format != 0 && "Given Vulkan format not handled!");
    GLuint image_handle;
    glGenTextures(1, &image_handle);
    glBindTexture(GL_TEXTURE_2D, image_handle);
    glTextureStorageMem2DEXT(image_handle, 1, format, vk_handle.width, vk_handle.height,
                             memory_handle, 0);
    //glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle_mask);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, swizzle_mask[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, swizzle_mask[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, swizzle_mask[2]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_A, swizzle_mask[3]);
    switch (usage) {
        case swapchain_usage::LEFT_SWAPCHAIN: {
            _m_eye_swapchains[0].push_back(image_handle);
            break;
        }
        case swapchain_usage::RIGHT_SWAPCHAIN: {
            _m_eye_swapchains[1].push_back(image_handle);
            break;
        }
        case swapchain_usage::LEFT_RENDER: {
            _m_eye_output_textures[0] = image_handle;
            break;
        }
        case swapchain_usage::RIGHT_RENDER: {
            _m_eye_output_textures[1] = image_handle;
            break;
        }
        default: {
            assert(false && "Invalid swapchain usage");
            break;
        }
    }
}*/

void timewarp_gl::vulkanGL_interop_buffer(const data_format::vk_buffer_handle& vk_buffer_handle,
                                          data_format::swapchain_usage usage) {
    [[maybe_unused]] const bool gl_result = static_cast<bool>(eglMakeCurrent(display_, surface_,
                                                                             surface_,
                                                                             glcontext_));
    assert(gl_result && "glXMakeCurrent should not fail");
    EGLClientBuffer native_buffer = NULL;
    if (vk_buffer_handle.ahardware_buffer == NULL)
        spdlog::get("illixr")->debug("NULL ahardware buffer ..");
    native_buffer = eglGetNativeClientBufferANDROID(vk_buffer_handle.ahardware_buffer);
    if (native_buffer == NULL)
        spdlog::get("illixr")->debug("NATIVE BUFFER IS NULL");
    AHardwareBuffer_Desc desc;
    AHardwareBuffer_describe(vk_buffer_handle.ahardware_buffer, &desc);
    EGLint attrs[] = {
            EGL_IMAGE_PRESERVED_KHR,
            EGL_TRUE,
//                EGL_PROTECTED_CONTENT_EXT,
            (desc.usage & AHARDWAREBUFFER_USAGE_PROTECTED_CONTENT) ? EGL_TRUE : EGL_FALSE,
            EGL_NONE,
            EGL_NONE,
            EGL_NONE,
    };
    EGLenum source = EGL_NATIVE_BUFFER_ANDROID;
    EGLImageKHR image = eglCreateImageKHR(display_, EGL_NO_CONTEXT, source, native_buffer, attrs);
    GLint swizzle_mask[4] = {GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA};
    GLuint format = convert_vk_format_to_gl(vk_buffer_handle.format, swizzle_mask);
    assert(format != 0 && "Given VK format not handled!");
    spdlog::get("illixr")->debug("Format : %d", format);
    GLuint image_handle;
    glGenTextures(1, &image_handle);
    glBindTexture(GL_TEXTURE_2D, image_handle);
    //glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES)image);
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
    glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC) eglGetProcAddress(
            "glEGLImageTargetTexture2DOES");
    glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, (GLeglImageOES) image); //Try the EXT alternative
    //glEGLImageTargetTexStorageEXT(GL_TEXTURE_2D, image, NULL);
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR)
        spdlog::get("illixr")->debug("error %d", err);
    spdlog::get("illixr")->debug("NO ERROR");
    //Alternate GL_TEXTURE_2D
    //glTextureStorageMem2DEXT(image_handle, 1, format, vk_buffer_handle.width, vk_buffer_handle.height, native_buffer, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, swizzle_mask[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, swizzle_mask[1]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, swizzle_mask[2]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_A, swizzle_mask[3]);
//        _m_swapchain[swapchain_index].push_back(image_handle);
    spdlog::get("illixr")->debug("SWITCH");
    switch (usage) {
        case data_format::swapchain_usage::LEFT_SWAPCHAIN: {
            spdlog::get("illixr")->debug("Pushed LEFT_SWAPCHAIN");
            eye_swapchains_[0].push_back(image_handle);
            break;
        }
        case data_format::swapchain_usage::RIGHT_SWAPCHAIN: {
            spdlog::get("illixr")->debug("Pushed RIGHT_SWAPCHAIN");
            eye_swapchains_[1].push_back(image_handle);
            break;
        }
        case data_format::swapchain_usage::LEFT_RENDER: {
            spdlog::get("illixr")->debug("Pushed LEFT_RENDER");
            eye_output_textures_[0] = image_handle;
            break;
        }
        case data_format::swapchain_usage::RIGHT_RENDER: {
            spdlog::get("illixr")->debug("Pushed RIGHT_RENDER");
            eye_output_textures_[1] = image_handle;
            break;
        }
        default: {
            assert(false && "Invalid swapchain usage");
            break;
        }
    }
}

#ifdef ILLIXR_MONADO
void timewarp_gl::import_vulkan_semaphore(const semaphore_handle& vk_handle) {
        [[maybe_unused]] const bool gl_result = static_cast<bool>(eglMakeCurrent(dpy, surface, surface, glc));
        assert(gl_result && "glXMakeCurrent should not fail");
        //assert(GLEW_EXT_memory_object_fd && "[timewarp_gl] Missing object memory extensions for Vulkan-GL interop");

        //GLuint semaphore_handle;
        //glGenSemaphoresEXT(1, &semaphore_handle);
        //glImportSemaphoreFdEXT(semaphore_handle, GL_HANDLE_TYPE_OPAQUE_FD_EXT, vk_handle.vk_handle);

//                        switch (vk_handle.usage) {
//                        case semaphore_usage::LEFT_RENDER_COMPLETE: {
//                                _m_semaphores[0] = semaphore_handle;
//                                break;
//                            }
//                            case semaphore_usage::RIGHT_RENDER_COMPLETE: {
//                                _m_semaphores[1] = semaphore_handle;
//                                break;
//                            }
//                            default: {
//                                assert(false && "Invalid semaphore usage");
//                                break;
//                            }
//                        }
    }
#endif


void timewarp_gl::build_timewarp(HMD::hmd_info_t& hmdInfo) {
    // Calculate the number of vertices+indices in the distortion mesh.
    num_distortion_vertices_ = (hmdInfo.eyeTilesHigh + 1) * (hmdInfo.eyeTilesWide + 1);
    num_distortion_indices_ = hmdInfo.eyeTilesHigh * hmdInfo.eyeTilesWide * 6;

    // Allocate memory for the elements/indices array.
    distortion_indices_.resize(num_distortion_indices_);

    // This is just a simple grid/plane index array, nothing fancy.
    // Same for both eye distortions, too!
    for (int y = 0; y < hmdInfo.eyeTilesHigh; y++) {
        for (int x = 0; x < hmdInfo.eyeTilesWide; x++) {
            const int offset = (y * hmdInfo.eyeTilesWide + x) * 6;

            distortion_indices_[offset + 0] = (GLuint) ((y + 0) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 0));
            distortion_indices_[offset + 1] = (GLuint) ((y + 1) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 0));
            distortion_indices_[offset + 2] = (GLuint) ((y + 0) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 1));

            distortion_indices_[offset + 3] = (GLuint) ((y + 0) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 1));
            distortion_indices_[offset + 4] = (GLuint) ((y + 1) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 0));
            distortion_indices_[offset + 5] = (GLuint) ((y + 1) * (hmdInfo.eyeTilesWide + 1) +
                                                        (x + 1));
        }
    }

    // There are `num_distortion_vertices` distortion coordinates for each color channel (3) of each eye (2).
    // These are NOT the coordinates of the distorted vertices. They are *coefficients* that will be used to
    // offset the UV coordinates of the distortion mesh.
    std::array<std::array<std::vector<HMD::mesh_coord2d_t>, HMD::NUM_COLOR_CHANNELS>, HMD::NUM_EYES> distort_coords;
    for (auto& eye_coords: distort_coords) {
        for (auto& channel_coords: eye_coords) {
            channel_coords.resize(num_distortion_vertices_);
        }
    }
    HMD::build_distortion_meshes(distort_coords, hmdInfo);

    // Allocate memory for position and UV CPU buffers.
    const std::size_t num_elems_pos_uv = HMD::NUM_EYES * num_distortion_vertices_;
    distortion_positions_.resize(num_elems_pos_uv);
    distortion_uv0_.resize(num_elems_pos_uv);
    distortion_uv1_.resize(num_elems_pos_uv);
    distortion_uv2_.resize(num_elems_pos_uv);

    for (int eye = 0; eye < HMD::NUM_EYES; eye++) {
        for (int y = 0; y <= hmdInfo.eyeTilesHigh; y++) {
            for (int x = 0; x <= hmdInfo.eyeTilesWide; x++) {
                const int index = y * (hmdInfo.eyeTilesWide + 1) + x;

                // Set the physical distortion mesh coordinates. These are rectangular/gridlike, not distorted.
                // The distortion is handled by the UVs, not the actual mesh coordinates!
                distortion_positions_[eye * num_distortion_vertices_ + index].x =
                        (-1.0f + 2.0f * ((float) x / hmdInfo.eyeTilesWide));
                distortion_positions_[eye * num_distortion_vertices_ + index].y =
                        (-1.0f +
                         2.0f * ((hmdInfo.eyeTilesHigh - (float) y) / hmdInfo.eyeTilesHigh) *
                         ((float) (hmdInfo.eyeTilesHigh * hmdInfo.tilePixelsHigh) /
                          hmdInfo.displayPixelsHigh));
                distortion_positions_[eye * num_distortion_vertices_ + index].z = 0.0f;

                // Use the previously-calculated distort_coords to set the UVs on the distortion mesh
                distortion_uv0_[eye * num_distortion_vertices_ +
                                index].u = distort_coords[eye][0][index].x;
                distortion_uv0_[eye * num_distortion_vertices_ +
                                index].v = distort_coords[eye][0][index].y;
                distortion_uv1_[eye * num_distortion_vertices_ +
                                index].u = distort_coords[eye][1][index].x;
                distortion_uv1_[eye * num_distortion_vertices_ +
                                index].v = distort_coords[eye][1][index].y;
                distortion_uv2_[eye * num_distortion_vertices_ +
                                index].u = distort_coords[eye][2][index].x;
                distortion_uv2_[eye * num_distortion_vertices_ +
                                index].v = distort_coords[eye][2][index].y;
            }
        }
    }

    // Construct perspective projection matrix
    math_util::projection_fov(&basic_projection_, display_params::fov_x / 2.0f,
                              display_params::fov_x / 2.0f,
                              display_params::fov_y / 2.0f, display_params::fov_y / 2.0f,
                              rendering_params::near_z,
                              rendering_params::far_z);
}

/* Calculate timewarm transform from projection matrix, view matrix, etc */
void timewarp_gl::calculate_timeWarp_transform(Eigen::Matrix4f& transform,
                                               const Eigen::Matrix4f& renderProjectionMatrix,
                                               const Eigen::Matrix4f& renderViewMatrix,
                                               const Eigen::Matrix4f& newViewMatrix) {
    // Eigen stores matrices internally in column-major order.
    // However, the (i,j) accessors are row-major (i.e, the first argument
    // is which row, and the second argument is which column.)
    Eigen::Matrix4f texCoordProjection;
    texCoordProjection << 0.5f * renderProjectionMatrix(0, 0), 0.0f,
            0.5f * renderProjectionMatrix(0, 2) - 0.5f, 0.0f, 0.0f,
            0.5f * renderProjectionMatrix(1, 1), 0.5f * renderProjectionMatrix(1, 2) -
                                                 0.5f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;

    // Calculate the delta between the view matrix used for rendering and
    // a more recent or predicted view matrix based on new sensor input.
    Eigen::Matrix4f inverseRenderViewMatrix = renderViewMatrix.inverse();

    Eigen::Matrix4f deltaViewMatrix = inverseRenderViewMatrix * newViewMatrix;

    deltaViewMatrix(0, 3) = 0.0f;
    deltaViewMatrix(1, 3) = 0.0f;
    deltaViewMatrix(2, 3) = 0.0f;

    // Accumulate the transforms.
    transform = texCoordProjection * deltaViewMatrix;
}

// Get the estimated time of the next swap/next Vsync.
// This is an estimate, used to wait until *just* before vsync.
time_point timewarp_gl::get_next_swap_time_estimate() {
    return time_last_swap_ + display_params::period;
}

// Get the estimated amount of time to put the CPU thread to sleep,
// given a specified percentage of the total Vsync period to delay.
duration timewarp_gl::estimate_time_to_sleep(double framePercentage) {
    return std::chrono::duration_cast<duration>(
            (get_next_swap_time_estimate() - clock_->now()) * framePercentage);
}

threadloop::skip_option timewarp_gl::_p_should_skip() {
    using namespace std::chrono_literals;
// Sleep for approximately 90% of the time until the next vsync.
// Scheduling granularity can't be assumed to be super accurate here,
// so don't push your luck (i.e. don't wait too long....) Tradeoff with
// MTP here. More you wait, closer to the display sync you sample the pose.

    std::this_thread::sleep_for(estimate_time_to_sleep(DELAY_FRACTION));

    if (image_handles_ready_.load() && eyebuffer_.get_ro_nullable() != nullptr) {
//            if (!semaphore_handles_ready.load()) {
//                return skip_option::skip_and_yield;
//            }
        return skip_option::run;
    }
// Null means system is nothing has been pushed yet
// because not all components are initialized yet
    return skip_option::skip_and_yield;
}

void timewarp_gl::_p_thread_setup() {
// Wait a vsync for gldemo to go first.
// This first time_last_swap will be "out of phase" with actual vsync.
// The second one should be on the dot, since we don't exit the first until actual vsync.
    time_last_swap_ = clock_->now() + display_params::period;

// Generate reference HMD and physical body dimensions
    HMD::get_default_hmd_info(display_params::width_pixels, display_params::height_pixels,
                              display_params::width_meters,
                              display_params::height_meters, display_params::lens_separation,
                              display_params::meters_per_tan_angle, display_params::aberration,
                              hmd_info_
    );

// Construct timewarp meshes and other data
    build_timewarp(hmd_info_);

// includes setting swap interval
//cl->wait_monado();
//
#ifdef ILLIXR_MONADO
    sem_wait(&cl->sem_monado);
#else
    lock_->get_lock();

#endif
    [[maybe_unused]] const bool gl_result_0 = static_cast<bool>(eglMakeCurrent(display_, surface_,
                                                                               surface_,
                                                                               glcontext_));
    assert(gl_result_0 && "eglMakeCurrent should not fail");

// set swap interval for 1
//        glXSwapIntervalEXTProc glXSwapIntervalEXT = 0;
//        glXSwapIntervalEXT = (glXSwapIntervalEXTProc) glXGetProcAddressARB((const GLubyte*) "glXSwapIntervalEXT");
//        glXSwapIntervalEXT(xwin->dpy, xwin->win, 1);
    eglSwapInterval(display_,
                    1);

// Init and verify GLEW
//        glewExperimental      = GL_TRUE;
//        const GLenum glew_err = glewInit();
//        if (glew_err != GLEW_OK) {
//            std::cerr << "[timewarp_gl] GLEW Error: " << glewGetErrorString(glew_err) << std::endl;
//            ILLIXR::abort("[timewarp_gl] Failed to initialize GLEW");
//        }

//glEnable(GL_DEBUG_OUTPUT);

//        glDebugMessageCallback(MessageCallback, 0);

// Create and bind global VAO object
    glGenVertexArrays(1, &tw_vao_);
    glBindVertexArray(tw_vao_);

#ifdef USE_ALT_EYE_FORMAT
    timewarp_shader_program_ =
            init_and_link(timeWarpChromaticVertexProgramGLSL,
                          timeWarpChromaticFragmentProgramGLSL_Alternative);
#else
    timewarpShaderProgram = init_and_link(timeWarpChromaticVertexProgramGLSL, timeWarpChromaticFragmentProgramGLSL);
#endif
// Acquire attribute and uniform locations from the compiled and linked shader program

    distortion_pos_attr_ = glGetAttribLocation(timewarp_shader_program_, "vertexPosition");
    distortion_uv0_attr_ = glGetAttribLocation(timewarp_shader_program_, "vertexUv0");
    distortion_uv1_attr_ = glGetAttribLocation(timewarp_shader_program_, "vertexUv1");
    distortion_uv2_attr_ = glGetAttribLocation(timewarp_shader_program_, "vertexUv2");

    tw_start_transform_unif_ = glGetUniformLocation(timewarp_shader_program_,
                                                    "TimeWarpStartTransform");
    tw_end_transform_unif_ = glGetUniformLocation(timewarp_shader_program_, "TimeWarpEndTransform");
    tw_eye_index_unif_ = glGetUniformLocation(timewarp_shader_program_, "ArrayLayer");

    eye_sampler_0_ = glGetUniformLocation(timewarp_shader_program_, "Texture[0]");
    eye_sampler_1_ = glGetUniformLocation(timewarp_shader_program_, "Texture[1]");
    flip_y_unif_ = glGetUniformLocation(timewarp_shader_program_, "flipY");
// Config distortion mesh position vbo
    glGenBuffers(1, &distortion_positions_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, distortion_positions_vbo_);

    const std::size_t num_elems_pos_uv = HMD::NUM_EYES * num_distortion_vertices_;

    HMD::mesh_coord3d_t* const distortion_positions_data = distortion_positions_.data();
    assert(distortion_positions_data != nullptr && "Timewarp allocation should not fail");
    glBufferData(GL_ARRAY_BUFFER, num_elems_pos_uv * sizeof(HMD::mesh_coord3d_t),
                 distortion_positions_data,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(distortion_pos_attr_,
                          3, GL_FLOAT, GL_FALSE, 0, 0);
// glEnableVertexAttribArray(distortion_pos_attr);

// Config distortion uv0 vbo
    glGenBuffers(1, &distortion_uv0_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, distortion_uv0_vbo_);

    HMD::uv_coord_t* const distortion_uv0_data = distortion_uv0_.data();
    assert(distortion_uv0_data != nullptr && "Timewarp allocation should not fail");
    glBufferData(GL_ARRAY_BUFFER, num_elems_pos_uv * sizeof(HMD::uv_coord_t), distortion_uv0_data,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(distortion_uv0_attr_,
                          2, GL_FLOAT, GL_FALSE, 0, 0);
// glEnableVertexAttribArray(distortion_uv0_attr);

// Config distortion uv1 vbo
    glGenBuffers(1, &distortion_uv1_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, distortion_uv1_vbo_);

    HMD::uv_coord_t* const distortion_uv1_data = distortion_uv1_.data();
    assert(distortion_uv1_data != nullptr && "Timewarp allocation should not fail");
    glBufferData(GL_ARRAY_BUFFER, num_elems_pos_uv * sizeof(HMD::uv_coord_t), distortion_uv1_data,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(distortion_uv1_attr_,
                          2, GL_FLOAT, GL_FALSE, 0, 0);
// glEnableVertexAttribArray(distortion_uv1_attr);

// Config distortion uv2 vbo
    glGenBuffers(1, &distortion_uv2_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, distortion_uv2_vbo_);

    HMD::uv_coord_t* const distortion_uv2_data = distortion_uv2_.data();
    assert(distortion_uv2_data != nullptr && "Timewarp allocation should not fail");
    glBufferData(GL_ARRAY_BUFFER, num_elems_pos_uv * sizeof(HMD::uv_coord_t), distortion_uv2_data,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(distortion_uv2_attr_,
                          2, GL_FLOAT, GL_FALSE, 0, 0);
// glEnableVertexAttribArray(distortion_uv2_attr);

// Config distortion mesh indices vbo
    glGenBuffers(1, &distortion_indices_vbo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, distortion_indices_vbo_);

    GLuint* const distortion_indices_data = distortion_indices_.data();
    assert(distortion_indices_data != nullptr && "Timewarp allocation should not fail");
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_distortion_indices_ * sizeof(GLuint),
                 distortion_indices_data, GL_STATIC_DRAW);

    if (enable_offload_) {
// Config PBO for texture image collection
        glGenBuffers(1, &PBO_buffer_);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, PBO_buffer_);
        glBufferData(GL_PIXEL_PACK_BUFFER,
                     display_params::width_pixels * display_params::height_pixels * 3, 0,
                     GL_STREAM_DRAW);
    }

    [[maybe_unused]] const bool gl_result_1 = static_cast<bool>(eglMakeCurrent(display_, NULL, NULL,
                                                                               nullptr));
    assert(gl_result_1 && "eglMakeCurrent should not fail");
    spdlog::get("illixr")->debug("Android api level : %d", __ANDROID_API__);

#ifdef ILLIXR_MONADO
    sem_post(&cl->sem_illixr);
#else
    lock_->release_lock();
#endif
}

void timewarp_gl::_p_one_iteration() {
//cl->wait_monado();
    auto start = std::chrono::high_resolution_clock::now();

#ifdef ILLIXR_MONADO
    sem_wait(&cl->sem_monado);
#else
    lock_->get_lock();
#endif
//LOGT("ITERATION STARTED");
    [[maybe_unused]] const bool gl_result = static_cast<bool>(eglMakeCurrent(display_, surface_,
                                                                             surface_,
                                                                             glcontext_));
    assert(gl_result && "eglMakeCurrent should not fail");
//
//        glBindFramebuffer(GL_FRAMEBUFFER, 0);
//        glViewport(0, 0, display_params::width_pixels, display_params::height_pixels);
//        glClearColor(0, 0, 1, 0);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
//        glDepthFunc(GL_LEQUAL);

    if (!rendering_ready_) {
//LOGT("not ready now ..");
        while (!image_handles_ready_);
        assert(image_handles_ready_);
        spdlog::get("illixr")->debug("ready now ..");
//           client_backend = _m_image_handles[0][0].type;
        for (int eye = 0; eye < 2; eye++) {
            uint32_t num_images = eye_image_handles_[eye][0].num_images;
            for (uint32_t image_index = 0; image_index < num_images; image_index++) {
                data_format::image_handle image = eye_image_handles_[eye][image_index];
                if (client_backend_ == data_format::graphics_api::OPENGL) {
                    eye_swapchains_[eye].push_back(image.gl_handle);
                } else {
                    spdlog::get("illixr")->debug("CONVERTING IMAGES TO GL");
//VulkanGLInterop(_m_image_handles[eye][image_index].vk_handle, eye);
//ImportVulkanImage(image.vk_handle, image.usage);
                    vulkanGL_interop_buffer(image.vk_buffer_handle, image.usage);
                }
            }
        }
//swapchain_ready = true;
// If we're using Monado, we need to import the eye output textures to render to.
// Otherwise with native, we can directly create the texture
        for (
                int eye = 0;
                eye < 2; eye++) {
#ifdef ILLIXR_MONADO
            image_handle image = eye_output_handles_[eye];
                // ImportVulkanImage(image.vk_handle, image.usage);
                VulkanGLInteropBuffer(image.vk_buffer_handle, image.usage);
                LOGT("IMPORT VULKAN SEMAPHORE");
                // Each eye also has an associated semaphore
               // ImportVulkanSemaphore(semaphore_handles_[eye]);
#else
            GLuint eye_output_texture;
            glGenTextures(1, &eye_output_texture);
            eye_output_textures_[eye] =
                    eye_output_texture;
            glBindTexture(GL_TEXTURE_2D, eye_output_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, display_params::width_pixels * 0.5f,
                         display_params::height_pixels, 0, GL_RGB, GL_FLOAT, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
#endif

// Once the eye output textures are created, we bind them to the framebuffer
            GLuint framebuffer;
            glGenFramebuffers(1, &framebuffer);
            eye_framebuffers_[eye] = framebuffer;

            glBindFramebuffer(GL_FRAMEBUFFER, eye_framebuffers_[eye]);
            glBindTexture(GL_TEXTURE_2D, eye_output_textures_[eye]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                   eye_output_textures_[eye], 0);

            uint32_t attachment = GL_COLOR_ATTACHMENT0;
            glDrawBuffers(1, &attachment);
        }
        spdlog::get("illixr")->debug("Rendering ready");
        rendering_ready_ = true;
    }

    switchboard::ptr<const data_format::rendered_frame> most_recent_frame = eyebuffer_.get_ro();

// Use the timewarp program
    glUseProgram(timewarp_shader_program_);

// Generate "starting" view matrix, from the pose sampled at the time of rendering the frame
    Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();
    viewMatrix.block(0, 0, 3,
                     3) = most_recent_frame->render_pose.pose.orientation.toRotationMatrix();

// We simulate two asynchronous view matrices, one at the beginning of
// display refresh, and one at the end of display refresh. The
// distortion shader will lerp between these two predictive view
// transformations as it renders across the horizontal view,
// compensating for display panel refresh delay (wow!)
    Eigen::Matrix4f viewMatrixBegin = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f viewMatrixEnd = Eigen::Matrix4f::Identity();
//LOGT("Disable warp %d", disable_warp);
    const data_format::fast_pose_type latest_pose = disable_warp_ ? most_recent_frame->render_pose
                                                                  : pose_prediction_->get_fast_pose();
    viewMatrixBegin.block(0, 0, 3, 3) = latest_pose.pose.orientation.toRotationMatrix();

// TODO: We set the "end" pose to the same as the beginning pose, but this really should be the pose for
// `display_period` later
    viewMatrixEnd = viewMatrixBegin;

// Calculate the timewarp transformation matrices. These are a product
// of the last-known-good view matrix and the predictive transforms.
    Eigen::Matrix4f timeWarpStartTransform4x4;
    Eigen::Matrix4f timeWarpEndTransform4x4;

// Calculate timewarp transforms using predictive view transforms
    calculate_timeWarp_transform(timeWarpStartTransform4x4, basic_projection_, viewMatrix,
                                 viewMatrixBegin);
    calculate_timeWarp_transform(timeWarpEndTransform4x4, basic_projection_, viewMatrix,
                                 viewMatrixEnd);

    glUniformMatrix4fv(tw_start_transform_unif_,
                       1, GL_FALSE, (GLfloat*) (timeWarpStartTransform4x4.data()));
    glUniformMatrix4fv(tw_end_transform_unif_,
                       1, GL_FALSE, (GLfloat*) (timeWarpEndTransform4x4.data()));
    glUniform1i(flip_y_unif_, client_backend_ == data_format::graphics_api::VULKAN);
// Debugging aid, toggle switch for rendering in the fragment shader
    glUniform1i(glGetUniformLocation(timewarp_shader_program_, "ArrayIndex"),
                0);
    glUniform1i(eye_sampler_0_,
                0);

#ifndef USE_ALT_EYE_FORMAT
    // Bind the shared texture handle
    glBindTexture(GL_TEXTURE_2D_ARRAY, most_recent_frame->texture_handle);
#endif

    glBindVertexArray(tw_vao_);

    auto gpu_start_wall_time = clock_->now();

//GLuint   query;
    GLuint64 elapsed_time = 0;

//        glGenQueries(1, &query);
//        glBeginQuery(GL_TIME_ELAPSED, query);

// Loop over each eye.
    for (int eye = 0; eye < HMD::NUM_EYES; eye++) {
        glBindFramebuffer(GL_FRAMEBUFFER, eye_framebuffers_[eye]);
        glViewport(0, 0, display_params::width_pixels * 0.5, display_params::height_pixels);
        glClearColor(1.0, 0.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        glDepthFunc(GL_LEQUAL);
#ifdef USE_ALT_EYE_FORMAT // If we're using Monado-style buffers we need to rebind eyebuffers.... eugh!
        [[maybe_unused]] const bool isTexture = static_cast<bool>(glIsTexture(
                eye_swapchains_[eye][most_recent_frame->swapchain_indices[eye]]));
        assert(isTexture && "The requested image is not a texture!");
        glBindTexture(GL_TEXTURE_2D,
                      eye_swapchains_[eye][most_recent_frame->swapchain_indices[eye]]);
#endif

// The distortion_positions_vbo GPU buffer already contains
// the distortion mesh for both eyes! They are contiguously
// laid out in GPU memory. Therefore, on each eye render,
// we set the attribute pointer to be offset by the full
// eye's distortion mesh size, rendering the correct eye mesh
// to that region of the screen. This prevents re-uploading
// GPU data for each eye.
        glBindBuffer(GL_ARRAY_BUFFER, distortion_positions_vbo_);
        glVertexAttribPointer(distortion_pos_attr_,
                              3, GL_FLOAT, GL_FALSE, 0,
                              (void*) (eye * num_distortion_vertices_
                                       * sizeof(HMD::mesh_coord3d_t)));
        glEnableVertexAttribArray(distortion_pos_attr_);

// We do the exact same thing for the UV GPU memory.
        glBindBuffer(GL_ARRAY_BUFFER, distortion_uv0_vbo_);
        glVertexAttribPointer(distortion_uv0_attr_,
                              2, GL_FLOAT, GL_FALSE, 0,
                              (void*) (eye * num_distortion_vertices_
                                       * sizeof(HMD::mesh_coord2d_t)));
        glEnableVertexAttribArray(distortion_uv0_attr_);

// We do the exact same thing for the UV GPU memory.
        glBindBuffer(GL_ARRAY_BUFFER, distortion_uv1_vbo_);
        glVertexAttribPointer(distortion_uv1_attr_,
                              2, GL_FLOAT, GL_FALSE, 0,
                              ((void*) (eye * num_distortion_vertices_
                                        * sizeof(HMD::mesh_coord2d_t))));
        glEnableVertexAttribArray(distortion_uv1_attr_);

// We do the exact same thing for the UV GPU memory.
        glBindBuffer(GL_ARRAY_BUFFER, distortion_uv2_vbo_);
        glVertexAttribPointer(distortion_uv2_attr_,
                              2, GL_FLOAT, GL_FALSE, 0,
                              (void*) (eye * num_distortion_vertices_
                                       * sizeof(HMD::mesh_coord2d_t)));
        glEnableVertexAttribArray(distortion_uv2_attr_);

#ifndef USE_ALT_EYE_FORMAT // If we are using normal ILLIXR-format eyebuffers
        // Specify which layer of the eye texture we're going to be using.
        // Each eye has its own layer.
        glUniform1i(tw_eye_index_unif, eye);
#endif

// Interestingly, the element index buffer is identical for both eyes, and is
// reused for both eyes. Therefore glDrawElements can be immediately called,
// with the UV and position buffers correctly offset.
        glDrawElements(GL_TRIANGLES, num_distortion_indices_, GL_UNSIGNED_INT, (void*) 0);

#ifdef ILLIXR_MONADO
        //GLenum dstLayout = GL_LAYOUT_SHADER_READ_ONLY_EXT;
        //glSignalSemaphoreEXT(_m_semaphores[eye], 0, nullptr, 1, &_m_eye_output_textures[eye], &dstLayout);
#endif
    }

    glFinish();
//LOGT("ITERATION finished");

    illixr_signal_.put(illixr_signal_.allocate<data_format::illixr_signal>(
            data_format::illixr_signal{++prev_counter_}));
//        glEndQuery(GL_TIME_ELAPSED);

#ifndef NDEBUG
    const duration time_since_render = clock_->now() - most_recent_frame->render_time;

    if (log_count > LOG_PERIOD) {
        const double time_since_render_ms_d = duration_to_double<std::milli>(time_since_render);
        std::cout << "\033[1;36m[TIMEWARP]\033[0m Time since render: " << time_since_render_ms_d
                  << "ms" << std::endl;
    }

    if (time_since_render > display_params::period) {
        std::cout << "\033[0;31m[TIMEWARP: CRITICAL]\033[0m Stale frame!" << std::endl;
    }
#endif
    // Call Hologram
    hologram_.put(hologram_.allocate<data_format::hologram_input>(
            data_format::hologram_input{++hologram_seq_}));

// Call swap buffers; when vsync is enabled, this will return to the
// CPU thread once the buffers have been successfully swapped.
    [[maybe_unused]] time_point time_before_swap = clock_->now();

//LOGT("EGL Swap buffer ...");
//eglSwapBuffers(xwin->display, xwin->surface);
// If we're not using Monado, we want to composite the left and right buffers into one
#ifndef ILLIXR_MONADO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, display_params::width_pixels, display_params::height_pixels);
// Blit the left and right color buffers onto the default color buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, eye_framebuffers_[0]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, display_params::width_pixels * 0.5, display_params::height_pixels, 0, 0,
                      display_params::width_pixels * 0.5, display_params::height_pixels,
                      GL_COLOR_BUFFER_BIT,
                      GL_NEAREST);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, eye_framebuffers_[1]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, display_params::width_pixels * 0.5, display_params::height_pixels,
                      display_params::width_pixels * 0.5, 0, display_params::width_pixels,
                      display_params::height_pixels, GL_COLOR_BUFFER_BIT, GL_NEAREST);

//glBindFramebuffer(GL_FRAMEBUFFER, 0);
    eglSwapBuffers(display_, surface_);
#endif
// The swap time needs to be obtained and published as soon as possible
    time_last_swap_ = clock_->now();
    [[maybe_unused]] time_point time_after_swap = time_last_swap_;

//LOGT("add vsync estimate");
// Now that we have the most recent swap time, we can publish the new estimate.
    vsync_estimate_.put(vsync_estimate_.allocate<switchboard::event_wrapper<time_point>>(
            get_next_swap_time_estimate()));

    std::chrono::nanoseconds imu_to_display =
            time_last_swap_.time_since_epoch() - latest_pose.pose.sensor_time.time_since_epoch();
//        auto duration_mtp =  std::chrono::duration_cast<std::chrono::microseconds>(time_last_swap.time_since_epoch() - latest_pose.pose.sensor_time.time_since_epoch());
//        sl->write_duration("mtp", duration_to_double(duration_mtp));
    std::chrono::nanoseconds predict_to_display =
            time_last_swap_ - latest_pose.predict_computed_time;
    std::chrono::nanoseconds render_to_display = time_last_swap_ - most_recent_frame->render_time;
    //LOGT("mtp logger ..");
    mtp_logger_.log(record{mtp_record,
                           {
                                   {iteration_no},
                                   {time_last_swap_},
                                   {imu_to_display},
                                   {predict_to_display},
                                   {render_to_display},
                           }});

    if (enable_offload_) {
// Read texture image from texture buffer
        GLubyte* image = read_texture_image();

        // Publish image and pose
        offload_data_.put(offload_data_.allocate<data_format::texture_pose>(
                data_format::texture_pose{offload_duration_, image, time_last_swap_,
                                          latest_pose.pose.position, latest_pose.pose.orientation,
                                          most_recent_frame->render_pose.pose.orientation}));
    }

#ifndef NDEBUG
    if (log_count > LOG_PERIOD) {
        const double time_swap = duration_to_double<std::milli>(time_after_swap - time_before_swap);
        const double latency_mtd = duration_to_double<std::milli>(imu_to_display);
        const double latency_ptd = duration_to_double<std::milli>(predict_to_display);
        const double latency_rtd = duration_to_double<std::milli>(render_to_display);
        const time_point time_next_swap = get_next_swap_time_estimate();
        const double timewarp_estimate = duration_to_double<std::milli>(
                time_next_swap - time_last_swap_);

        std::cout << "\033[1;36m[TIMEWARP]\033[0m Swap time: " << time_swap << "ms" << std::endl
                  << "\033[1;36m[TIMEWARP]\033[0m Motion-to-display latency: " << latency_mtd
                  << "ms" << std::endl
                  << "\033[1;36m[TIMEWARP]\033[0m Prediction-to-display latency: " << latency_ptd
                  << "ms" << std::endl
                  << "\033[1;36m[TIMEWARP]\033[0m Render-to-display latency: " << latency_rtd
                  << "ms" << std::endl
                  << "Next swap in: " << timewarp_estimate << "ms in the future" <<
                  std::endl;
    }
#endif

// retrieving the recorded elapsed time
// wait until the query result is available
//        int done = 0;
//        glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);
//
//        while (!done) {
//            std::this_thread::yield();
//            glGetQueryObjectiv(query, GL_QUERY_RESULT_AVAILABLE, &done);
//        }
//
//        // get the query result
//        glGetQueryObjectui64v(query, GL_QUERY_RESULT, &elapsed_time);
//LOGT("egl context.");
    [[maybe_unused]] const bool gl_result_1 = static_cast<bool>(eglMakeCurrent(display_, NULL, NULL,
                                                                               nullptr));
    assert(gl_result_1 && "eglMakeCurrent should not fail");
//cl->release_lock();
#ifdef ILLIXR_MONADO
    sem_post(&cl->sem_illixr);
#else
    lock_->release_lock();
#endif
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    spdlog::get("illixr")->debug("duration: %f", duration_to_double(duration));
//        sl->write_duration("timewarp", duration_to_double(duration));
    //LOGT("Lock released ..");
    timewarp_gpu_logger_.log(record{timewarp_gpu_record,
                                    {
                                            {iteration_no},
                                            {gpu_start_wall_time},
                                            {clock_->now()},
                                            {std::chrono::nanoseconds(elapsed_time)},
                                    }});

#ifndef NDEBUG
    if (log_count > LOG_PERIOD) {
        log_count = 0;
    } else {
        log_count++;
    }
#endif
}

PLUGIN_MAIN(timewarp_gl)
