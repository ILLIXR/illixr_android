#pragma once

#include "illixr/data_format/opencv_data_types.hpp"
#include "illixr/data_format/pose.hpp"
#include "illixr/data_format/misc.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/threadloop.hpp"

#include <media/NdkImageReader.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>

#define CARMA_SOURCE_PASSTHROUGH 0
#define POSITION_LEFT 0
#define POSITION_RIGHT 1

namespace ILLIXR {

class quest_camera : public threadloop {
public:
    [[maybe_unused]]quest_camera(const std::string& name_, phonebook *pb_);
    ~quest_camera() override;
    void _p_one_iteration() override;

    static void on_disconnected(void *context, ACameraDevice *device);
    static void on_error(void *context, ACameraDevice *device, int error);
    static void on_session_active(void *context, ACameraCaptureSession *session);
    static void on_session_ready(void *context, ACameraCaptureSession *session);
    static void on_session_closed(void *context, ACameraCaptureSession *session);
    static void on_capture_failed(void *context, ACameraCaptureSession *session,
                                  ACaptureRequest *request, ACameraCaptureFailure *failure);
    static void on_capture_sequence_completed(void *context, ACameraCaptureSession *session,
                                              int sequenceId, int64_t frameNumber) {
        (void)context;
        (void)session;
        (void)sequenceId;
        (void)frameNumber;
    }

    static void on_capture_sequence_aborted(void *context, ACameraCaptureSession *session,
                                            int sequenceId) {
        (void)context;
        (void)session;
        (void)sequenceId;
    }

    static void on_capture_completed(void *context, ACameraCaptureSession *session,
                                     ACaptureRequest *request, const ACameraMetadata *result) {
        (void)context;
        (void)session;
        (void)request;
        (void)result;
    }
    skip_option _p_should_skip() override;
private:
    void image_callback(AImageReader *reader, int position);
    void image_callback_left(void *context, AImageReader *reader);
    void image_callback_right(void *context, AImageReader *reader);

    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<const relative_clock> clock_;
    //switchboard::writer<data_format::pose_type> pose_;
    switchboard::writer<data_format::binocular_cam_type> cam_;
    std::string left_camera_id_;
    std::string right_camera_id_;

    static ullong cam_proc_time_;
    static bool left_camera_ready_ = false;
    static bool right_camera_ready_ = false;
    static cv::Mat last_image_left_;
    static cv::Mat last_image_right_;
    ACameraManager* camera_manager_ = nullptr;
    ACameraDevice* left_camera_device_ = nullptr;
    ACameraDevice* right_camera_device_ = nullptr;

    ACameraDevice_stateCallbacks camera_device_callbacks_ = {
            .context = nullptr,
            .onDisconnected = on_disconnected,
            .onError = on_error,
    };

    ACameraCaptureSession_stateCallbacks session_state_callbacks_{
            .context = nullptr,
            .onClosed = on_session_closed,
            .onReady = on_session_ready,
            .onActive = on_session_active,
    };

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

    AImageReader* create_jpeg_reader();
};

} // ILLIXR
