#include "plugin.hpp"

#include <array>
#include <spdlog/spdlog.h>
#include <vector>

//#include <jni>

using namespace ILLIXR;

typedef struct {
    XrSwapchain swapchain;
    uint32_t width;
    uint32_t height;
    uint32_t imageCount;
    XrSwapchainImageOpenGLESKHR* images;
} SwapchainInfo;

typedef struct {
    XrInstance instance;
    XrSystemId system_id;
    XrSession session;
    XrSpace local_space;
    XrSpace view_space;
    XrSessionState session_state;
    XrBool32 session_running;

    // EGL/Graphics
    EGLDisplay egl_display;
    EGLContext egl_context;
    EGLConfig egl_config;

    XrViewConfigurationView view_configs[2];
    XrView views[2];

    SwapchainInfo swapchains[2];

    GLuint framebuffers[2];
} app_state;

static app_state app = {nullptr};

// Identity pose helper
static XrPosef identity_pose() {
    XrPosef pose = {{0}};
    pose.position.y = 1.5f;
    pose.orientation.w = 1.0f;
    return pose;
}
/*{
template <typename Result>
Result JNI_CheckResult(Result result, const char* function, JNIEnv* env) {
    if constexpr (std::is_pointer<Result>::value) {
        if (result == nullptr) {
            ALOGE("JNI function failed %s", function);
            abort();
        }
    }
    if (env->ExceptionCheck() == JNI_TRUE) {
        ALOGE("JNI function caused a java exception %s", function);
        abort();
    }
    return result;
}

bool CheckUseScenePermission(JNIEnv* env, jobject activityObject) {
#define JNI_CHECK_RESULT(func) JNI_CheckResult(func, #func, env);
    jstring strPermission = JNI_CHECK_RESULT(env->NewStringUTF("com.oculus.permission.USE_SCENE"));
    jclass clsActivity = JNI_CHECK_RESULT(env->FindClass("android/app/Activity"));
    jmethodID methodCheckSelfPermission = JNI_CHECK_RESULT(
            env->GetMethodID(clsActivity, "checkSelfPermission", "(Ljava/lang/String;)I"));
    jint intPermissionResult = JNI_CHECK_RESULT(
            env->CallIntMethod(activityObject, methodCheckSelfPermission, strPermission));
    jclass clsPackageManager =
    JNI_CHECK_RESULT(env->FindClass("android/content/pm/PackageManager"));
    jfieldID fidPermissionGranted =
    JNI_CHECK_RESULT(env->GetStaticFieldID(clsPackageManager, "PERMISSION_GRANTED", "I"));
    jint intPermissionGranted =
    JNI_CHECK_RESULT(env->GetStaticIntField(clsPackageManager, fidPermissionGranted));
    env->DeleteLocalRef(strPermission);
    return intPermissionResult == intPermissionGranted;
#undef JNI_CHECK_RESULT
}

}*/
[[maybe_unused]] oxr_interface::oxr_interface(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , switchboard_{phonebook_->lookup_impl<switchboard>()}
        , app_{switchboard_->get_android_app()}
        , clock_{phonebook_->lookup_impl<relative_clock>()}
        , pose_writer_{switchboard_->get_writer<data_format::pose_type>("xr_pose")}
        , frame_reader_{switchboard_->get_buffered_reader<data_format::dual_frames>("unity_rendered_frame")} {
    renderer_ = std::make_unique<stereo_renderer>(QUEST3_EYE_WIDTH, QUEST3_EYE_HEIGHT);
    spdlog::get("illixr")->debug("OXR starting");
    init_xr();
    spdlog::get("illixr")->debug("IXR_INIT done");
    create_session();
    spdlog::get("illixr")->debug("OXR create_session done");
    create_swapchains();
    spdlog::get("illixr")->debug("OXR create_swapchains done");
}

void oxr_interface::init_xr() {
    //JNIEnv* java_env;
    //(*app_->activity->vm).AttachCurrentThread(&java_env, nullptr);
    //std::stringstream tss(switchboard_->get_env("REQUIRED_OXR_EXTENSIONS"));
    //while(tss.good()) {
    //    std::string substr;
    //    getline(tss, substr, ',');
    //    required_extensions_.emplace_back(substr.c_str());
    //}
    PFN_xrInitializeLoaderKHR init_loader;
    xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction*)&init_loader);
    if (init_loader != nullptr) {
        XrLoaderInitInfoAndroidKHR init_info_android = {XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
        init_info_android.applicationVM = app_->activity->vm;
        init_info_android.applicationContext = app_->activity->clazz;
        init_loader((XrLoaderInitInfoBaseHeaderKHR*)&init_info_android);
    }

    const char* extensions[] = {XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
                                XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME
    };
    /*
    //XrResult result;
    //PFN_xrEnumerateApiLayerProperties enumerate_layer_properties;
    //result = xrGetInstanceProcAddr(XR_NULL_HANDLE,
    //                               "xrEnumerateApiLayerProperties",
    //                               (PFN_xrVoidFunction*)&enumerate_layer_properties);
    //if (result != XR_SUCCESS) {
    //    spdlog::get("illixr")->error("Failed to get XrEnumerateApiLayerProperties pointer");
    //}
    //uint32_t layer_count = 0;
    //OXR(enumerate_layer_properties(0, &layer_count, nullptr));
    //layer_properties_ = std::vector<XrApiLayerProperties>(layer_count, {XR_TYPE_API_LAYER_PROPERTIES});
    //OXR(enumerate_layer_properties(layer_count, &layer_count, layer_properties_.data()));
    //for (const auto& layer : layer_properties_) {
    //    spdlog::get("illixr")->debug("Found layer {}", layer.layerName);
    //}
//    char** required_extensions = new char*[required_extensions_.size()];
//    for (int i = 0; i < required_extensions_.size(); i++) {
//        required_extensions[i] = required_extensions_[i];
//    }
    //const uint32_t num_req_extensions = required_extensions_.size();
    //const uint32_t num_req_extensions = sizeof(required_extensions_) / sizeof(required_extensions_[0]);
    //uint32_t num_output_extensions = 0;
    //OXR(xrEnumerateInstanceExtensionProperties(nullptr, 0, &num_output_extensions, nullptr));
    //spdlog::get("illixr")->debug("Found {} extensions", num_output_extensions);

    //extension_properties_ = std::vector<XrExtensionProperties>(num_output_extensions, {XR_TYPE_EXTENSION_PROPERTIES});

    //OXR(xrEnumerateInstanceExtensionProperties(NULL, num_output_extensions, &num_output_extensions, extension_properties_.data()));
    //for (const auto& ext : extension_properties_) {
    //    spdlog::get("illixr")->debug("Found extension {}", ext.extensionName);
    //}
    //for (auto i = 0; i < num_req_extensions; i++) {
    //    bool found = false;
    //    for (auto j = 0; j < num_output_extensions; j++) {
    //        if(!strcmp(required_extensions_[i], extension_properties_[j].extensionName)) {
    //            spdlog::get("illixr")->debug("Found required extension {}", required_extensions_[i]);
    //            found = true;
    //            break;
    //        }
    //    }
    //    if (!found) {
    //        spdlog::get("illixr")->error("Failed to locate required extension {}", required_extensions_[i]);
    //    }
    //}*/

    XrInstanceCreateInfoAndroidKHR android_info = {XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
    android_info.applicationVM = app_->activity->vm;
    android_info.applicationActivity = app_->activity->clazz;

    XrInstanceCreateInfo create_info = {XR_TYPE_INSTANCE_CREATE_INFO};
    create_info.next = &android_info;
    create_info.enabledExtensionCount = 2;
    create_info.enabledExtensionNames = extensions;
    strcpy(create_info.applicationInfo.applicationName, "ILLIXR_oxr");
    create_info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    
    XrResult result;
    OXR(result = xrCreateInstance(&create_info, &app.instance))
    if (result != XR_SUCCESS) {
        spdlog::get("illixr")->error("Failed to create xrInstance {}", static_cast<int>(result));
    }

    instance_ = app.instance;
    XrInstanceProperties instance_info = {XR_TYPE_INSTANCE_PROPERTIES};
    OXR(xrGetInstanceProperties(app.instance, &instance_info))
    spdlog::get("illixr")->debug("Runtime: {}  Version{}.{}.{}", instance_info.runtimeName,
                 XR_VERSION_MAJOR(instance_info.runtimeVersion),
                 XR_VERSION_MINOR(instance_info.runtimeVersion),
                 XR_VERSION_PATCH(instance_info.runtimeVersion));

    XrSystemGetInfo system_get_info = {XR_TYPE_SYSTEM_GET_INFO};
    system_get_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    OXR(result = xrGetSystem(app.instance, &system_get_info, &app.system_id))
    if (result != XR_SUCCESS) {
        if (result == XR_ERROR_FORM_FACTOR_UNAVAILABLE) {
            spdlog::get("illixr")->error("\"Failed to get system; the specified form factor is not available. Is your headset connected?");
        } else {
            spdlog::get("illixr")->error("xrGetSystem failed {}", static_cast<int>(result));
        }
    }
//
    XrSystemProperties system_properties = {XR_TYPE_SYSTEM_PROPERTIES};
    OXR(xrGetSystemProperties(app.instance, app.system_id, &system_properties))
    spdlog::get("illixr")->debug("System Properties: Name={} VendorId={}",
                                  system_properties.systemName,
                                  system_properties.vendorId);

    spdlog::get("illixr")->debug("System Graphics Properties: MaxWidth={} MaxHeight={} MaxLayers={}",
                                  system_properties.graphicsProperties.maxSwapchainImageWidth,
                                  system_properties.graphicsProperties.maxSwapchainImageHeight,
                                  system_properties.graphicsProperties.maxLayerCount);

    spdlog::get("illixr")->debug("System Tracking Properties: OrientationTracking={} PositionTracking={}",
                                  system_properties.trackingProperties.orientationTracking ? "True" : "False",
                                  system_properties.trackingProperties.positionTracking ? "True" : "False");

    //assert(App::kMaxLayerCount <= systemProperties.graphicsProperties.maxLayerCount);
}

void oxr_interface::create_session() {
    app.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(app.egl_display, nullptr, nullptr);

    const EGLint config_attribs[] = {
            EGL_RED_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_BLUE_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 24,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
            EGL_NONE
    };

    EGLint num_configs;
    eglChooseConfig(app.egl_display, config_attribs, &app.egl_config, 1, &num_configs);

    // CRITICAL FIX: Create context WITHOUT shared context first
    // This makes it the "base" context that can be shared
    const EGLint context_attribs[] = {
            EGL_CONTEXT_CLIENT_VERSION, 3,
            EGL_NONE
    };

    app.egl_context = eglCreateContext(app.egl_display, app.egl_config,
                                      EGL_NO_CONTEXT, context_attribs);

    app.egl_context = eglCreateContext(app.egl_display, app.egl_config,
                                       EGL_NO_CONTEXT, context_attribs);  // No share context

    if (app.egl_context == EGL_NO_CONTEXT) {
        EGLint error = eglGetError();
        spdlog::get("illixr")->error("Failed to create EGL context: 0x{:X}", error);
        throw std::runtime_error("Failed to create EGL context");
    }
    // Create a dummy surface for the context
    const EGLint surface_attribs[] = {
            EGL_WIDTH, 16,
            EGL_HEIGHT, 16,
            EGL_NONE
    };
    EGLSurface dummy_surface = eglCreatePbufferSurface(app.egl_display, app.egl_config, surface_attribs);
    //eglMakeCurrent(app.egl_display, dummy_surface, dummy_surface, app.egl_context);
// Make context current initially
    if (!eglMakeCurrent(app.egl_display, dummy_surface, dummy_surface, app.egl_context)) {
        EGLint error = eglGetError();
        spdlog::get("illixr")->error("Failed to make initial context current: 0x{:X}", error);
        throw std::runtime_error("Failed to make initial context current");
    }
    spdlog::get("illixr")->info("EGL context created and made current: {}", (void*)app.egl_context);

    // Get graphics requirements (required even if not rendering)
    DECL_PFN(xrGetOpenGLESGraphicsRequirementsKHR);
    INIT_PFN(xrGetOpenGLESGraphicsRequirementsKHR,xrGetOpenGLESGraphicsRequirementsKHR)

    XrGraphicsRequirementsOpenGLESKHR graphics_requirements = {XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
    OXR(xrGetOpenGLESGraphicsRequirementsKHR(app.instance, app.system_id, &graphics_requirements))
    spdlog::get("illixr")->debug("Graphics requirements: min %d.%d, max %d.%d",
                                  XR_VERSION_MAJOR(graphics_requirements.minApiVersionSupported),
                                  XR_VERSION_MINOR(graphics_requirements.minApiVersionSupported),
                                  XR_VERSION_MAJOR(graphics_requirements.maxApiVersionSupported),
                                  XR_VERSION_MINOR(graphics_requirements.maxApiVersionSupported));

    XrGraphicsBindingOpenGLESAndroidKHR graphics_binding = {
            XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
    graphics_binding.display = app.egl_display;
    graphics_binding.config = app.egl_config;
    graphics_binding.context = app.egl_context;  
        
    XrSessionCreateInfo session_create_info = {XR_TYPE_SESSION_CREATE_INFO};
    session_create_info.next = &graphics_binding;
    session_create_info.systemId = app.system_id;
    
    XrResult result;
    OXR(result = xrCreateSession(app.instance, &session_create_info, &app.session))
    if (result != XR_SUCCESS) {
        spdlog::get("illixr")->error("Failed to create XR session: {}.", static_cast<int>(result));
    } else {
        spdlog::get("illixr")->debug("Created session");
    }
    session_ = app.session;

    uint32_t viewCount;
    OXR(xrEnumerateViewConfigurationViews(app.instance, app.system_id,
                                          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &viewCount, nullptr))

    app.view_configs[0].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
    app.view_configs[1].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
    OXR(xrEnumerateViewConfigurationViews(app.instance, app.system_id,
                                          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &viewCount, app.view_configs))

    spdlog::get("illixr")->debug("View 0: %dx%d, View 1: %dx%d",
                                 app.view_configs[0].recommendedImageRectWidth,
                                 app.view_configs[0].recommendedImageRectHeight,
                                 app.view_configs[1].recommendedImageRectWidth,
                                 app.view_configs[1].recommendedImageRectHeight);

    XrReferenceSpaceCreateInfo reference_space_info = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    reference_space_info.poseInReferenceSpace = identity_pose();

    reference_space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    OXR(xrCreateReferenceSpace(app.session, &reference_space_info, &app.local_space))

    reference_space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    OXR(xrCreateReferenceSpace(app.session, &reference_space_info, &app.view_space))
}

void oxr_interface::_p_one_iteration() {
    poll_events();
    run_frame();
}

oxr_interface::~oxr_interface() {
    for (auto & swapchain : app.swapchains) {
        if (swapchain.swapchain != XR_NULL_HANDLE) {
            xrDestroySwapchain(swapchain.swapchain);
        }
        if (swapchain.images) {
            free(swapchain.images);
        }
    }

    if (app.framebuffers[0]) {
        glDeleteFramebuffers(2, app.framebuffers);
    }    if (app.view_space != XR_NULL_HANDLE) {
        xrDestroySpace(app.view_space);
    }
    if (app.local_space != XR_NULL_HANDLE) {
        xrDestroySpace(app.local_space);
    }
    if (app.session != XR_NULL_HANDLE) {
        xrDestroySession(app.session);
    }
    if (app.instance != XR_NULL_HANDLE) {
        xrDestroyInstance(app.instance);
    }
    if (app.egl_context != EGL_NO_CONTEXT) {
        eglDestroyContext(app.egl_display, app.egl_context);
    }
    if (app.egl_display != EGL_NO_DISPLAY) {
        eglTerminate(app.egl_display);
    }
}

void oxr_interface::poll_events() {
    XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};

    while (xrPollEvent(app.instance, &event) == XR_SUCCESS) {
        switch (event.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto* stateEvent = (XrEventDataSessionStateChanged*)&event;
                app.session_state = stateEvent->state;

                spdlog::get("illixr")->debug("Session state changed to: {}", static_cast<int>(app.session_state));

                if (app.session_state == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo begin_info = {XR_TYPE_SESSION_BEGIN_INFO};
                    begin_info.primaryViewConfigurationType =
                            XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    xrBeginSession(app.session, &begin_info);
                    app.session_running = XR_TRUE;
                    spdlog::get("illixr")->debug("Session started");
                }
                else if (app.session_state == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(app.session);
                    app.session_running = XR_FALSE;
                }
                break;
            }
            default:
                break;
        }
        event.type = XR_TYPE_EVENT_DATA_BUFFER;
    }
}

XrResult oxr_interface::get_head_pose(XrTime time, XrPosef* outPose, XrBool32* outValid) {
    XrSpaceLocation location = {XR_TYPE_SPACE_LOCATION};
    //spdlog::get("illixr")->debug("Getting pose");
    OXR(xrLocateSpace(app.view_space, app.local_space, time, &location))

    // Check if pose is valid and tracked
    XrSpaceLocationFlags validFlags =
            XR_SPACE_LOCATION_POSITION_VALID_BIT |
            XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;

    *outValid = (location.locationFlags & validFlags) == validFlags;
    *outPose = location.pose;

    return XR_SUCCESS;
}

void oxr_interface::run_frame() {
    if (!app.session_running) 
        return;
    // IMPORTANT: Make context current at the start of EVERY frame
    // MediaCodec decoder runs on another thread and may have changed it
    if (eglGetCurrentContext() != app.egl_context) {
        spdlog::get("illixr")->debug("Re-binding GL context for render frame");

        if (!eglMakeCurrent(app.egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, app.egl_context)) {
            EGLint error = eglGetError();
            spdlog::get("illixr")->error("Failed to bind GL context in run_frame: 0x{:X}", error);
            return;
        }
    }
    // Wait for frame
    XrFrameWaitInfo wait_info = {XR_TYPE_FRAME_WAIT_INFO};
    XrFrameState frame_state = {XR_TYPE_FRAME_STATE};
    xrWaitFrame(app.session, &wait_info, &frame_state);

    // Begin frame
    XrFrameBeginInfo begin_info = {XR_TYPE_FRAME_BEGIN_INFO};
    xrBeginFrame(app.session, &begin_info);

    XrCompositionLayerProjection projectionLayer = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    XrCompositionLayerProjectionView projectionViews[2] = {
            {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
            {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}
    };
    int layerCount = 0;
    const XrCompositionLayerBaseHeader* layers[1] = {nullptr};

    if (frame_state.shouldRender) {
        // Locate views (eye positions)
        XrViewState viewState = {XR_TYPE_VIEW_STATE};
        XrViewLocateInfo viewLocateInfo = {XR_TYPE_VIEW_LOCATE_INFO};
        viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        viewLocateInfo.displayTime = frame_state.predictedDisplayTime;
        viewLocateInfo.space = app.local_space;

        // Get head pose at predicted display time
        XrPosef headPose;
        XrBool32 poseValid;
        if (XR_SUCCEEDED(get_head_pose(frame_state.predictedDisplayTime,
                                       &headPose, &poseValid))) {
            if (poseValid) {
                //spdlog::get("illixr")->debug("pose valid, writing");
                pose_writer_.put(pose_writer_.allocate<data_format::pose_type>(
                        data_format::pose_type{clock_->now(),
                                               {headPose.position.x, headPose.position.y, headPose.position.z},
                                               {headPose.orientation.w, headPose.orientation.x,
                                                headPose.orientation.y, headPose.orientation.z}
                        }));

            } else {
                spdlog::get("illixr")->debug("Head pose not tracked");
            }
        }

        uint32_t viewCount = 2;
        app.views[0].type = XR_TYPE_VIEW;
        app.views[1].type = XR_TYPE_VIEW;
        xrLocateViews(app.session, &viewLocateInfo, &viewState, 2, &viewCount, app.views);

        // Example: Create some dummy image data (replace with your actual images)
        const long long now = clock_->now().time_since_epoch().count() + 20000000;
        current_frames_ = nullptr;
        while (clock_->now().time_since_epoch().count() < now) {
            if (frame_reader_.size() > 0) {
                current_frames_ = frame_reader_.dequeue();
                spdlog::get("illixr")->debug("OXR got frame");
                
                break;
            }
        }
        if (current_frames_ != nullptr) {
            renderer_->receive_dual_frame(current_frames_);

            // OpenXR render loop for each eye (Quest 3 supports 72Hz and 90Hz)
            for (int eye = 0; eye < 2; eye++) {
                // Get MVP matrix from OpenXR for this eye
                float mvp[16];
                get_openxr_projection_matrix(eye, mvp);

                // Bind framebuffer for this eye
                bind_eye_framebuffer(eye);

                // Clear and setup viewport
                glViewport(0, 0, static_cast<int>(app.swapchains[eye].width), static_cast<int>(app.swapchains[eye].height));
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                // Render video frame for this eye
                if (renderer_->render_eye(eye, mvp)) {
                    // Successfully rendered frame for this eye
                } else {
                    // No frame ready yet - render blank or last frame
                    spdlog::get("illixr")->debug("No frame ready for eye {}", eye);
                }

                // Release swapchain image for this eye
                release_swapchain_image(eye);
            }

            /*uint8_t* leftImage = static_cast<uint8_t*>(malloc(imageSize));
            uint8_t* rightImage = static_cast<uint8_t*>(malloc(imageSize));

            // Fill with solid colors (replace this with your actual image data)
            for (size_t i = 0; i < imageSize; i += 4) {
                leftImage[i] = 255;     // R
                leftImage[i + 1] = 0;     // G
                leftImage[i + 2] = 0;     // B
                leftImage[i + 3] = 255;   // A

                rightImage[i] = 0;      // R
                rightImage[i + 1] = 0;    // G
                rightImage[i + 2] = 255;  // B
                rightImage[i + 3] = 255;  // A
            }*/

            // Render your images to the swapchains
            //render_stereo_images(current_frames_->left_eye.data(), current_frames_->right_eye.data());

            //free(leftImage);
            //free(rightImage);

            // Set up projection layer
            for (int eye = 0; eye < 2; eye++) {
                projectionViews[eye].pose = app.views[eye].pose;
                projectionViews[eye].fov = app.views[eye].fov;
                projectionViews[eye].subImage.imageArrayIndex = 0;
                projectionViews[eye].subImage.swapchain = app.swapchains[eye].swapchain;
                projectionViews[eye].subImage.imageRect.offset.x = 0;
                projectionViews[eye].subImage.imageRect.offset.y = 0;
                projectionViews[eye].subImage.imageRect.extent.width = static_cast<int>(app.swapchains[eye].width);
                projectionViews[eye].subImage.imageRect.extent.height = static_cast<int>(app.swapchains[eye].height);
                projectionViews[eye].subImage.imageArrayIndex = 0;
            }

            projectionLayer.space = app.local_space;
            projectionLayer.viewCount = 2;
            projectionLayer.views = projectionViews;

            layers[0] = (XrCompositionLayerBaseHeader*) &projectionLayer;
            layerCount = 1;
        }
    }
    // End frame (normally you'd submit layers here)
    XrFrameEndInfo endInfo = {XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = frame_state.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    endInfo.layerCount = layerCount;
    endInfo.layers = layers;
    xrEndFrame(app.session, &endInfo);
}
/*
void oxr_interface::upload_image_to_texture(GLuint texture, uint32_t width, uint32_t height,
                             const uint8_t* imageData) {
    glBindTexture(GL_TEXTURE_2D, texture);

    // Upload RGBA8 data (assuming your uint8_t* is in RGBA format)
    // If your data is RGB, change GL_RGBA to GL_RGB and adjust accordingly
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, imageData);

    // Optional: set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
}


void oxr_interface::render_stereo_images(const uint8_t* left_eye, const uint8_t* right_eye) {
    for (int eye = 0; eye < 2; eye++) {
        SwapchainInfo* sc = &app.swapchains[eye];

        // Acquire swapchain image
        XrSwapchainImageAcquireInfo acquireInfo = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
        uint32_t imageIndex;
        xrAcquireSwapchainImage(sc->swapchain, &acquireInfo, &imageIndex);

        // Wait for the image to be ready
        XrSwapchainImageWaitInfo waitInfo = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
        waitInfo.timeout = XR_INFINITE_DURATION;
        xrWaitSwapchainImage(sc->swapchain, &waitInfo);

        // Get the OpenGL texture ID from the swapchain
        GLuint texture = sc->images[imageIndex].image;

        // Upload your image data to this texture
        const uint8_t* imageData = (eye == 0) ? left_eye : right_eye;
        upload_image_to_texture(texture, sc->width, sc->height, imageData);

        // Release the swapchain image
        XrSwapchainImageReleaseInfo releaseInfo = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
        xrReleaseSwapchainImage(sc->swapchain, &releaseInfo);
    }
}*/

void oxr_interface::create_swapchains() {
    // Create one swapchain per eye
    for (int eye = 0; eye < 2; eye++) {
        SwapchainInfo* sc = &app.swapchains[eye];

        // Use your actual resolution: 1032x1104 per eye
        // Note: This is half of Quest 3's native 2064x2208
        sc->width = 1032;   // Your input resolution
        sc->height = 1104;  // Your input resolution

        // You could also use recommended resolution if you want full native:
        // sc->width = app.view_configs[eye].recommendedImageRectWidth;
        // sc->height = app.view_configs[eye].recommendedImageRectHeight;

        XrSwapchainCreateInfo swapchainInfo = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
        swapchainInfo.arraySize = 1;
        swapchainInfo.format = GL_RGBA8;  // or GL_SRGB8_ALPHA8 for sRGB
        swapchainInfo.width = sc->width;
        swapchainInfo.height = sc->height;
        swapchainInfo.mipCount = 1;
        swapchainInfo.faceCount = 1;
        swapchainInfo.sampleCount = 1;
        swapchainInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT |
                                   XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;

        OXR(xrCreateSwapchain(app.session, &swapchainInfo, &sc->swapchain))

        // Get swapchain images
        OXR(xrEnumerateSwapchainImages(sc->swapchain, 0, &sc->imageCount, NULL))
        sc->images = static_cast<XrSwapchainImageOpenGLESKHR*>(malloc(
                sc->imageCount * sizeof(XrSwapchainImageOpenGLESKHR)));
        for (uint32_t i = 0; i < sc->imageCount; i++) {
            sc->images[i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR;
            sc->images[i].next = NULL;
        }
        OXR(xrEnumerateSwapchainImages(sc->swapchain, sc->imageCount,
                                       &sc->imageCount,
                                       (XrSwapchainImageBaseHeader*)sc->images))

        spdlog::get("illixr")->info("Eye {} swapchain: {}x{} with {} images",
                                    eye, sc->width, sc->height, sc->imageCount);

        // Log all swapchain texture IDs
        for (uint32_t i = 0; i < sc->imageCount; i++) {
            spdlog::get("illixr")->info("  Swapchain image {}: texture ID {}",
                                        i, sc->images[i].image);
        }
    }

    // Create framebuffers for each eye
    glGenFramebuffers(2, app.framebuffers);

    // Check if framebuffers were created successfully
    GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR) {
        spdlog::get("illixr")->error("glGenFramebuffers failed: GL error 0x{:X}", gl_error);
        //return XR_ERROR_RUNTIME_FAILURE;
    }

    spdlog::get("illixr")->info("Created framebuffers: left={}, right={}",
                                app.framebuffers[0], app.framebuffers[1]);

    // Verify framebuffers are valid
    if (app.framebuffers[0] == 0 || app.framebuffers[1] == 0) {
        spdlog::get("illixr")->error("Invalid framebuffer IDs generated");
        //return XR_ERROR_RUNTIME_FAILURE;
    }

    //return XR_SUCCESS;
}

// Helper to convert XrFovf and XrPosef to a projection-view matrix
void oxr_interface::get_openxr_projection_matrix(int eye, float* mvp) {
// For fullscreen video quad rendering, use identity matrix
    // The video renderer will handle the quad placement
    identity_matrix(mvp);

    // If you want to use actual OpenXR projection for 3D positioning:
    // XrFovf fov = app.views[eye].fov;
    // XrPosef pose = app.views[eye].pose;
    // create_projection_fov(projection, fov, 0.05f, 100.0f);
    // create_view_matrix(view, pose);
    // matrix_multiply(mvp, projection, view);
}

// Bind the framebuffer for the specified eye's current swapchain image
void oxr_interface::bind_eye_framebuffer(int eye) {
    SwapchainInfo* sc = &app.swapchains[eye];

    // Don't try to change context - it should already be correct
    // MediaCodec runs on a different thread with its own context
    EGLContext current_context = eglGetCurrentContext();

    if (current_context == EGL_NO_CONTEXT) {
        spdlog::get("illixr")->error("No EGL context current at all!");
        return;
    }

    if (current_context != app.egl_context) {
        spdlog::get("illixr")->warn("Using different context than expected");
        spdlog::get("illixr")->warn("Current: {}, Expected: {}",
                                    (void*)current_context, (void*)app.egl_context);
        // Continue anyway - might be MediaCodec's shared context
    }

    // Clear any previous OpenGL errors
    while (glGetError() != GL_NO_ERROR);

    // Acquire the next swapchain image
    XrSwapchainImageAcquireInfo acquire_info = {XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    uint32_t image_index;
    XrResult result = xrAcquireSwapchainImage(sc->swapchain, &acquire_info, &image_index);
    if (result != XR_SUCCESS) {
        spdlog::get("illixr")->error("Failed to acquire swapchain image for eye {}: {}",
                                     eye, static_cast<int>(result));
        return;
    }

    // Wait for the image to be ready
    XrSwapchainImageWaitInfo wait_info = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wait_info.timeout = XR_INFINITE_DURATION;
    result = xrWaitSwapchainImage(sc->swapchain, &wait_info);
    if (result != XR_SUCCESS) {
        spdlog::get("illixr")->error("Failed to wait for swapchain image for eye {}: {}",
                                     eye, static_cast<int>(result));
        return;
    }

    GLuint texture_id = sc->images[image_index].image;

    if (texture_id == 0) {
        spdlog::get("illixr")->error("Swapchain returned invalid texture ID 0 for eye {}", eye);
        return;
    }

    spdlog::get("illixr")->debug("Eye {} using swapchain image {} (texture ID: {})",
                                 eye, image_index, texture_id);

    // Bind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, app.framebuffers[eye]);
    GLenum gl_error = glGetError();
    if (gl_error != GL_NO_ERROR) {
        spdlog::get("illixr")->error("glBindFramebuffer failed for eye {}: GL error 0x{:X}",
                                     eye, gl_error);
        return;
    }

    // Attach the swapchain image as the color attachment
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, texture_id, 0);
    gl_error = glGetError();
    if (gl_error != GL_NO_ERROR) {
        spdlog::get("illixr")->error("glFramebufferTexture2D failed for eye {}: GL error 0x{:X}",
                                     eye, gl_error);
        return;
    }

    // Check framebuffer completeness
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    gl_error = glGetError();

    if (gl_error != GL_NO_ERROR) {
        spdlog::get("illixr")->error("GL error after glCheckFramebufferStatus: 0x{:X}", gl_error);
        return;
    }

    if (status == 0) {
        spdlog::get("illixr")->error("glCheckFramebufferStatus returned 0 for eye {}", eye);
        return;
    }

    if (status != GL_FRAMEBUFFER_COMPLETE) {
        const char* status_str = "UNKNOWN";
        switch (status) {
            case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                status_str = "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT"; break;
            case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                status_str = "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT"; break;
            case GL_FRAMEBUFFER_UNSUPPORTED:
                status_str = "GL_FRAMEBUFFER_UNSUPPORTED"; break;
            case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
                status_str = "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS"; break;
            case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
                status_str = "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE"; break;
        }
        spdlog::get("illixr")->error("Framebuffer incomplete for eye {}: {}", eye, status_str);
    } else {
        spdlog::get("illixr")->debug("✓ Framebuffer complete for eye {}", eye);
    }
}

// Release swapchain image after rendering
void oxr_interface::release_swapchain_image(int eye) {
    XrSwapchainImageReleaseInfo release_info = {XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    XrResult result = xrReleaseSwapchainImage(app.swapchains[eye].swapchain, &release_info);
    if (result != XR_SUCCESS) {
        spdlog::get("illixr")->error("Failed to release swapchain image for eye {}: {}",
                                     eye, static_cast<int>(result));
    }
}

void oxr_interface::create_projection_fov(float* result, XrFovf fov, float near_z, float far_z) {
    const float tan_left = tanf(fov.angleLeft);
    const float tan_right = tanf(fov.angleRight);
    const float tan_down = tanf(fov.angleDown);
    const float tan_up = tanf(fov.angleUp);

    const float tan_width = tan_right - tan_left;
    const float tan_height = tan_up - tan_down;

    memset(result, 0, 16 * sizeof(float));

    result[0] = 2.0f / tan_width;
    result[4] = 0.0f;
    result[8] = (tan_right + tan_left) / tan_width;
    result[12] = 0.0f;

    result[1] = 0.0f;
    result[5] = 2.0f / tan_height;
    result[9] = (tan_up + tan_down) / tan_height;
    result[13] = 0.0f;

    result[2] = 0.0f;
    result[6] = 0.0f;
    result[10] = -(far_z + near_z) / (far_z - near_z);
    result[14] = -(2.0f * far_z * near_z) / (far_z - near_z);

    result[3] = 0.0f;
    result[7] = 0.0f;
    result[11] = -1.0f;
    result[15] = 0.0f;
}
void oxr_interface::create_view_matrix(float* result, XrPosef pose) {
// Extract rotation (quaternion) and position
    XrQuaternionf q = pose.orientation;
    XrVector3f p = pose.position;

    // Convert quaternion to rotation matrix
    float x2 = q.x + q.x;
    float y2 = q.y + q.y;
    float z2 = q.z + q.z;
    float xx = q.x * x2;
    float xy = q.x * y2;
    float xz = q.x * z2;
    float yy = q.y * y2;
    float yz = q.y * z2;
    float zz = q.z * z2;
    float wx = q.w * x2;
    float wy = q.w * y2;
    float wz = q.w * z2;

    // Create inverse transform (view matrix)
    result[0] = 1.0f - (yy + zz);
    result[1] = xy + wz;
    result[2] = xz - wy;
    result[3] = 0.0f;

    result[4] = xy - wz;
    result[5] = 1.0f - (xx + zz);
    result[6] = yz + wx;
    result[7] = 0.0f;

    result[8] = xz + wy;
    result[9] = yz - wx;
    result[10] = 1.0f - (xx + yy);
    result[11] = 0.0f;

    // Apply inverse translation
    result[12] = -(result[0] * p.x + result[4] * p.y + result[8] * p.z);
    result[13] = -(result[1] * p.x + result[5] * p.y + result[9] * p.z);
    result[14] = -(result[2] * p.x + result[6] * p.y + result[10] * p.z);
    result[15] = 1.0f;
}

void oxr_interface::identity_matrix(float* result) {
    memset(result, 0, 16 * sizeof(float));
    result[0] = result[5] = result[10] = result[15] = 1.0f;
}
void oxr_interface::matrix_multiply(float* result, const float* a, const float* b) {
    float temp[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp[i * 4 + j] =
                    a[i * 4 + 0] * b[0 * 4 + j] +
                    a[i * 4 + 1] * b[1 * 4 + j] +
                    a[i * 4 + 2] * b[2 * 4 + j] +
                    a[i * 4 + 3] * b[3 * 4 + j];
        }
    }
    memcpy(result, temp, sizeof(float) * 16);
}
PLUGIN_MAIN(oxr_interface)
