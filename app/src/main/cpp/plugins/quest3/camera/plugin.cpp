#include "plugin.hpp"

#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraMetadataTags.h>
#include <condition_variable>
#include <jni.h>
#include <mutex>
#include <spdlog/spdlog.h>
#include <thread>

using namespace ILLIXR;
using namespace ILLIXR::data_format;

JavaVM* java_vm = nullptr;
jobject java_context = nullptr;

std::mutex mutex_;
bool have_camera_values = false;
std::condition_variable cond_var;
int camera_position_value = -1;
int camera_source_value = -1;


extern "C" {
    // when the library is loaded
    JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
        java_vm = vm;
        return JNI_VERSION_1_6;
    }

    // native callback for java to call
    JNIEXPORT void JNICALL
    Java_com_example_ILLIXR_QuestCamera_onCustomKeyValues(JNIEnv *env, jclass, jint position_value, jint source_value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            camera_position_value = position_value;
            camera_source_value = source_value;
            have_camera_values = true;
        }
        cond_var.notify_one();
    }

    JNIEXPORT void JNICALL
    Java_com_example_ILLIXR_NativeBridge_setContext(JNIEnv* env, jclass, jobject context) {
        if (java_context) {
            env->DeleteGlobalRef(java_context);
        }
        java_context = env->NewGlobalRef(context);
    }

    // call this from native code
    void queryKeysFromNative(const char* camera_id) {
        if (!java_vm || !java_context)
            return;

        JNIEnv* env = nullptr;
        java_vm->AttachCurrentThread(&env, nullptr);

        jclass helper_class = env->FindClass("com/example/ILLIXR$QuestCamera");
        if (!helper_class)
            return;
        jmethodID method = env->GetStaticMethodID(helper_class, "getCustomKeyValues",
                                                  "(Landroid/content/Context;Ljava/lang/String;)V");
        if (!method)
            return;
        jstring cam_id = env->NewStringUTF(camera_id);
        env-> CallStaticVoidMethod(helper_class, method, java_context, cam_id);
        env-> DeleteLocalRef(cam_id);
    }
}

quest_camera::quest_camera(const std::string& name_, ILLIXR::phonebook* pb_)
    : threadloop(name_, pb_)
    , switchboard_{phonebook_->lookup_impl<switchboard>()}
    , clock_{phonebook_->lookup_impl<relative_clock>()}
    //, pose_{switchboard_->get_writer<pose_type>("pose")}
    , cam_{switchboard_->get_writer<binocular_cam_type>("cam")}{
    camera_manager_ = ACameraManager_create();
    ACameraIdList* camera_ids = nullptr;
    ACameraManager_getCameraIdList(camera_manager_, &camera_ids);
    ACameraMetadata* metadata;

    for (int i = 0; i < camera_ids->numCameras; i++) {
        if (!left_camera_id_.empty() && !right_camera_id_.empty())
            break;
        ACameraManager_getCameraCharacteristics(camera_manager_, camera_ids->cameraIds[i], &metadata);
        ACameraMetadata_const_entry pixel_size = {0};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_SENSOR_INFO_PIXEL_ARRAY_SIZE, &pixel_size) != ACAMERA_OK) {

        }

        ACameraMetadata_const_entry pose_rotation = {0};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_POSE_ROTATION, &pose_rotation) != ACAMERA_OK) {

        }

        ACameraMetadata_const_entry pose_translation = {0};
        if (ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_POSE_TRANSLATION, &pose_translation) != ACAMERA_OK) {

        }
        have_camera_values = false;
        queryKeysFromNative(camera_ids->cameraIds[i]);
        // wait for return values
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var.wait(lock, []{ return have_camera_values; });
        if (camera_source_value == CARMA_SOURCE_PASSTHROUGH) {
            if (camera_position_value == POSITION_LEFT) {
                left_camera_id_ = camera_ids->cameraIds[i];
            } else if (camera_position_value == POSITION_RIGHT) {
                right_camera_id_ = camera_ids->cameraIds[i];
            } else if (camera_position_value < 0) {
                spdlog::get("illixr")->error("Invalid camera position from {}", camera_ids->cameraIds[i]);
            } else {
                continue;
            }
        } else if (camera_source_value < 0) {
            spdlog::get("illixr")->error("Invalid camera source from {}", camera_ids->cameraIds[i]);
        }
    }
    if (left_camera_id_.empty()) {
        spdlog::get("illixr")->error("Did not find left camera from quest.");
    }
    if (right_camera_id_.empty()) {
        spdlog::get("illixr")->error("Did not find right camera from quest.");
    }
    ACameraManager_deleteCameraIdList(camera_ids);
    ACameraManager_openCamera(camera_manager_, left_camera_id_.c_str(), &camera_device_callbacks_, &left_camera_device_);
    ACameraManager_openCamera(camera_manager_, right_camera_id_.c_str(), &camera_device_callbacks_, &right_camera_device_);
    ACameraDevice_createCaptureRequest(left_camera_device_, TEMPLATE_PREVIEW, &request_);
    ACameraDevice_createCaptureRequest(right_camera_device_, TEMPLATE_PREVIEW, &request_);
}

void quest_camera::_p_one_iteration() {
    auto start = std::chrono::high_resolution_clock::now();
    auto ts = std::chrono::system_clock::now().time_since_epoch().count();
    ullong cam_time = ts * 1000;
    cam_proc_time_ = 0;
    cv::Mat left_img = last_image_left_;
    cv::Mat right_img = last_image_right_;

}
threadloop:skip_option quest_camera::_p_should_skip() {
    //std::this_thread::sleep_for(
    if (right_camera_ready_ && left_camera_ready_)
        return skip_option::run;
    return skip_option::skip_and_yield;
}
void quest_camera::on_disconnected(void *context, ACameraDevice *device) {
    (void)context;
    (void)device;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("on_disconnected");
#endif
}
void quest_camera::on_error(void *context, ACameraDevice *device, int error) {
    (void)context;
    (void)device;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("error %d", error);
#endif
}
void quest_camera::on_session_active(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("on_session_active()");
#endif
}
void quest_camera::on_session_ready(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("on_session_ready()");
#endif
}
void quest_camera::on_session_closed(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("on_session_closed()");
#endif
}
void quest_camera::on_capture_failed(void *context, ACameraCaptureSession *session,
                              ACaptureRequest *request, ACameraCaptureFailure *failure) {
    (void)context;
    (void)session;
    (void)request;
    (void)failure;
#ifndef NDEBUG
    spdlog::get("illixr")->debug("on_capture_failed ");
#endif

}

quest_camera::~quest_camera() {
    ACameraManager_delete(camera_manager_);
    camera_manager_ = nullptr;
}

void quest_camera:image_callback(void *context, AImageReader *reader) {
    (void)context;
    AImage *image = nullptr;
    AImageReader_acquireNextImage(reader, &image);

}

AImageReader* quest_camera::create_jpeg_reader() {
    AImageReader *reader = nullptr;
    media_status_t status =
            AImageReader_new(IMAGE_WIDTH, IMAGE_HEIGHT, AIMAGE_FORMAT_YUV_420_888, 10, &reader);

    if (status != AMEDIA_OK) {
        spdlog::get("illixr")->error("Error with image reader");
        return nullptr;
    }

    AImageReader_ImageListener listener{
            .context = this,
            .onImageAvailable = image_callback,
    };

    AImageReader_setImageListener(reader, &listener);

    return reader;
}

void quest_camera::image_callback_left(void* context, AImageReader* reader) {
    (void) context;

}