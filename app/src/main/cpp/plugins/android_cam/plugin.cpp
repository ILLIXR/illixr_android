#include "plugin.hpp"

#include <opencv2/opencv.hpp> // Include OpenCV API
#include <opencv2/core/mat.hpp>

// ILLIXR includes
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <android/log.h>
#include <camera/NdkCameraMetadata.h>
#include <android/native_window_jni.h>


using namespace ILLIXR;

static cv::Mat last_image;
static bool img_ready = false;
static std::mutex mtx;
static double current_ts = 0;
bool once = false;


[[maybe_unused]] android_cam::android_cam(const std::string &name_, phonebook *pb_)
        : threadloop{name_, pb_}, switchboard_{pb_->lookup_impl<switchboard>()},
          clock_{pb->lookup_impl<RelativeClock>()},
          cam_{switchboard_->get_writer<data_format::cam_type>("cam")} {
    init_cam();
}

android_cam::~android_cam() {
    std::cout << "Deconstructing android_cam" << std::endl;
    ACameraCaptureSession_stopRepeating(capture_session_);
    ACameraCaptureSession_close(capture_session_);
    ACaptureSessionOutputContainer_free(outputs_);

    ACameraDevice_close(camera_device_);
    ACameraManager_delete(camera_manager_);
    camera_manager_ = nullptr;
    AImageReader_delete(image_reader_);
    image_reader_ = nullptr;
    ACaptureRequest_free(request_);
}

//Reference: https://github.com/sixo/native-camera/tree/93b05aec6d05604a314dc822b6b09a4cbc3d5104
void android_cam::image_callback(void *context, AImageReader *reader) {
    AImage *image = nullptr;
    AImageReader_acquireNextImage(reader, &image);
    if (image == nullptr) {
        ANDROID_LOG("IMage is null!");
        return;
    }

    // Try to process data without blocking the callback
    //std::thread processor([=](){
    uint8_t *rPixel;
    int32_t rLen;
    int32_t yPixelStride, yRowStride;
    AImage_getPlanePixelStride(image, 0, &yPixelStride);
    AImage_getPlaneRowStride(image, 0, &yRowStride);
    AImage_getPlaneData(image, 0, &rPixel, &rLen);
    uint8_t *data = new uint8_t[rLen];

    if (yPixelStride == 1) {
        for (int y = 0; y < IMAGE_HEIGHT; y++)
            memcpy(data + y * IMAGE_WIDTH, rPixel + y * yRowStride, IMAGE_WIDTH);
    }
    cv::Mat rawData(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, (uint8_t *) data);
    mtx.lock();
    current_ts = std::chrono::system_clock::now().time_since_epoch().count();
    //uint64_t name = (uint64_t)current_ts;
    // bool check = cv::imwrite(
    // "/sdcard/Android/data/com.example.native_activity/cam0/"+ std::to_string(name)+".png", rawData);
    // ANDROID_LOG("ANDROID CAM CHECK %d", check);
    last_image = rawData;
    mtx.unlock();
    img_ready = true;
    AImage_delete(image);
//        });
//        processor.detach();
}


AImageReader *android_cam::create_jpeg_reader() {
    AImageReader *reader = nullptr;
    media_status_t status =
            AImageReader_new(IMAGE_WIDTH, IMAGE_HEIGHT, AIMAGE_FORMAT_YUV_420_888, 10, &reader);

    if (status != AMEDIA_OK) {
        ANDROID_LOG("Error with image reader");
        return nullptr;
    }

    AImageReader_ImageListener listener{
            .context = this,
            .onImageAvailable = image_callback,
    };

    AImageReader_setImageListener(reader, &listener);

    return reader;
}

ANativeWindow* android_cam::create_surface(AImageReader *reader) {
    ANativeWindow *nativeWindow;
    AImageReader_getWindow(reader, &nativeWindow);
    return nativeWindow;
}

//Camera callbacks
void android_cam::on_disconnected(void *context, ACameraDevice *device) {
    (void)context;
    (void)device;
    ANDROID_LOG("on_disconnected");
}

void android_cam::on_error(void *context, ACameraDevice *device, int error) {
    (void)context;
    (void)device;
    ANDROID_LOG("error %d", error);
}

void android_cam::on_session_active(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
    ANDROID_LOG("on_session_active()");
}

void android_cam::on_session_ready(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
    ANDROID_LOG("on_session_ready()");
}

void android_cam::on_session_closed(void *context, ACameraCaptureSession *session) {
    (void)context;
    (void)session;
    ANDROID_LOG("on_session_closed()");
}

void android_cam::on_capture_failed(void *context, ACameraCaptureSession *session,
                                    ACaptureRequest *request, ACameraCaptureFailure *failure) {
    (void)context;
    (void)session;
    (void)request;
    (void)failure;
    ANDROID_LOG("on_capture_failed ");
}


std::string android_cam::get_back_facing_cam_id() {
    ACameraIdList *cameraIds = nullptr;
    ACameraManager_getCameraIdList(camera_manager_, &cameraIds);

    std::string backId;

    ANDROID_LOG("found camera count %d", cameraIds->numCameras);
    bool camera_found = false;
    for (int i = 0; i < cameraIds->numCameras; ++i) {
        const char *id = cameraIds->cameraIds[i];

        ACameraMetadata *metadataObj;
        ACameraManager_getCameraCharacteristics(camera_manager_, id, &metadataObj);
        ANDROID_LOG("get camera characteristics %d", i);

        ACameraMetadata_const_entry lensInfo = {0};
        ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_FACING, &lensInfo);
        ANDROID_LOG("lens info %d", i);

        auto facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
                lensInfo.data.u8[0]);

        ACameraMetadata_const_entry pose_reference = {0};
        ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_REFERENCE, &pose_reference);

        auto pose_ref = static_cast<acamera_metadata_enum_acamera_lens_pose_reference>(
                pose_reference.data.u8[0]);

        ANDROID_LOG("Pose reference is = %d and facing = %d", pose_ref, facing);
        // Found a back-facing camera?
        if (facing == ACAMERA_LENS_FACING_BACK) {
            backId = id;
            camera_found = true;
        }

        if (camera_found) {
            ACameraMetadata_const_entry intrinsics;
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_INTRINSIC_CALIBRATION,
                                          &intrinsics);
            for (int x = 0; x < intrinsics.count; ++x) {
                ANDROID_LOG("Intrinsics total = %d : %f : index %d", intrinsics.count,
                            intrinsics.data.f[x], x);
            }

            ACameraMetadata_const_entry distortion;
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_DISTORTION, &distortion);
            for (int x = 0; x < distortion.count; ++x) {
                ANDROID_LOG("distortion  = %d : %f : index %d", distortion.count, distortion.data.f[x], x);
            }

            ACameraMetadata_const_entry rotation;
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_ROTATION, &rotation);
            for (int x = 0; x < rotation.count; ++x) {
                ANDROID_LOG("rotation total = %d : %f : index %d", rotation.count, rotation.data.f[x], x);
            }

            ACameraMetadata_const_entry translation;
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_TRANSLATION, &translation);
            for (int x = 0; x < translation.count; ++x) {
                ANDROID_LOG("translation total = %d : %f : index %d", translation.count,
                            translation.data.f[x], x);
            }
            break;
        }


    }
    ANDROID_LOG("back camera id %s", backId.c_str());

    ACameraManager_deleteCameraIdList(cameraIds);
    return backId;
}

void android_cam::init_cam() {
    camera_manager_ = ACameraManager_create();
    auto id = get_back_facing_cam_id();
    ACameraManager_openCamera(camera_manager_, id.c_str(), &camera_device_callbacks_, &camera_device_);

    ACameraDevice_createCaptureRequest(camera_device_, TEMPLATE_PREVIEW, &request_);
    ACaptureSessionOutputContainer_create(&outputs_);
    image_reader_ = create_jpeg_reader();
    image_window_ = create_surface(image_reader_);
    ANativeWindow_acquire(image_window_);
    ACameraOutputTarget_create(image_window_, &image_target_);
    ACaptureRequest_addTarget(request_, image_target_);

    ACaptureSessionOutput_create(image_window_, &image_output_);
    ACaptureSessionOutputContainer_add(outputs_, image_output_);
    // Create the session
    ACameraDevice_createCaptureSession(camera_device_, outputs_, &session_state_callbacks_,
                                       &capture_session_);
    camera_status_t status = ACameraCaptureSession_setRepeatingRequest(capture_session_,
                                                                       &capture_callbacks_, 1,
                                                                       &request_, nullptr);
    ANDROID_LOG("ACameraCaptureSession_setRepeatingRequest status = %d", status);

}

/// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
/// _p_one_iteration() is called in a thread created by threadloop::start()
void android_cam::_p_one_iteration() {
    ANDROID_LOG("Android_cam started ..");
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    if (!img_ready)
        return;
    if (!clock_->is_started()) {
        return;
    }

    double ts = current_ts;
    ullong cam_time = static_cast<ullong>(ts * 1000);
    if (!first_cam_time_) {
        first_cam_time_ = cam_time;
        first_real_time_cam_ = clock_->now();
    }

    mtx.lock();
    cv::Mat ir_left = last_image;
    cv::Mat ir_right = last_image;

    time_point cam_time_point{
            *first_real_time_cam_ + std::chrono::nanoseconds(cam_time - *first_cam_time_)};
    ANDROID_LOG("TIME = %lf and cam_time %llu",
                duration2double(std::chrono::nanoseconds(cam_time - *first_cam_time_)), cam_time);
    cam_.put(cam_.allocate<data_format::cam_type>({cam_time_point, ir_left, ir_right}));
    mtx.unlock();
}

PLUGIN_MAIN(android_cam)

