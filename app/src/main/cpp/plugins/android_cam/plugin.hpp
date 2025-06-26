#pragma once

#include "illixr/data_format/opencv_data_types.hpp"
#include "illixr/data_format/misc.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

#include <media/NdkImageReader.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>

#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_cam", __VA_ARGS__))

namespace ILLIXR {

class android_cam : public threadloop {
public:
    [[maybe_unused]] android_cam(const std::string &name_, phonebook *pb_);

    ~android_cam() override;

    static void image_callback(void *context, AImageReader *reader);

    AImageReader *create_jpeg_reader();

    ANativeWindow *create_surface(AImageReader *reader);

    static void on_disconnected(void *context, ACameraDevice *device);

    static void on_error(void *context, ACameraDevice *device, int error);

    static void on_session_active(void *context, ACameraCaptureSession *session);

    static void on_session_ready(void *context, ACameraCaptureSession *session);

    static void on_session_closed(void *context, ACameraCaptureSession *session);


    static void on_capture_failed(void *context, ACameraCaptureSession *session,
                                  ACaptureRequest *request, ACameraCaptureFailure *failure);

    static void on_capture_sequence_completed(void *context, ACameraCaptureSession *session,
                                              int sequenceId, int64_t frameNumber) {}

    static void on_capture_sequence_aborted(void *context, ACameraCaptureSession *session,
                                            int sequenceId) {}

    static void on_capture_completed(void *context, ACameraCaptureSession *session,
                                     ACaptureRequest *request, const ACameraMetadata *result) {}

    std::string get_back_facing_cam_id();

    void init_cam();

    /// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
    /// _p_one_iteration() is called in a thread created by threadloop::start()
    void _p_one_iteration() override;

private:
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<const RelativeClock> clock_;
    switchboard::writer<data_format::cam_type> cam_;

    std::optional<ullong> first_cam_time_;
    std::optional<time_point> first_real_time_cam_;

    ACameraManager *camera_manager_ = nullptr;
    ACameraDevice *camera_device_ = nullptr;
    ANativeWindow *image_window_ = nullptr;
    ACameraOutputTarget *image_target_ = nullptr;
    AImageReader *image_reader_ = nullptr;
    ACaptureSessionOutput *image_output_ = nullptr;
    ACaptureRequest *request_ = nullptr;
    ACaptureSessionOutputContainer *outputs_ = nullptr;
    ACameraCaptureSession *capture_session_ = nullptr;
    static const int IMAGE_WIDTH = 752;
    static const int IMAGE_HEIGHT = 480;

    ACameraCaptureSession_captureCallbacks capture_callbacks_{
            .context = nullptr,
            .onCaptureStarted = nullptr,
            .onCaptureProgressed = nullptr,
            .onCaptureCompleted = on_capture_completed,
            .onCaptureFailed = on_capture_failed,
            .onCaptureSequenceCompleted = on_capture_sequence_completed,
            .onCaptureSequenceAborted = on_capture_sequence_aborted,
            .onCaptureBufferLost = nullptr,
    };

    ACameraCaptureSession_stateCallbacks session_state_callbacks_{
            .context = nullptr,
            .onClosed = on_session_closed,
            .onReady = on_session_ready,
            .onActive = on_session_active,
    };

    ACameraDevice_stateCallbacks camera_device_callbacks_ = {
            .context = nullptr,
            .onDisconnected = on_disconnected,
            .onError = on_error,
    };

};
}