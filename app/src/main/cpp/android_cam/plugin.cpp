#include <opencv2/opencv.hpp> // Include OpenCV API
#include <opencv2/core/mat.hpp>

// ILLIXR includes
#include "common/data_format.hpp"
#include "common/switchboard.hpp"
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <android/log.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraDevice.h>
#include <media/NdkImageReader.h>
#include <android/native_window_jni.h>

#define LOGA(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_cam", __VA_ARGS__))

using namespace ILLIXR;

class android_cam : public threadloop {
public:
    android_cam(std::string name_, phonebook* pb_)
            : threadloop{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , _m_clock{pb->lookup_impl<RelativeClock>()}
            , _m_cam{sb->get_writer<cam_type>("cam")}{
        initCam();
    }

    virtual ~android_cam() override {
        std::cout << "Deconstructing android_cam" << std::endl;
        ACameraCaptureSession_stopRepeating(captureSession);
        ACameraCaptureSession_close(captureSession);
        ACaptureSessionOutputContainer_free(outputs);

        ACameraDevice_close(cameraDevice);
        ACameraManager_delete(cameraManager);
        cameraManager = nullptr;
        AImageReader_delete(imageReader);
        imageReader = nullptr;
        ACaptureRequest_free(request);
    }

    //Reference: https://github.com/sixo/native-camera/tree/93b05aec6d05604a314dc822b6b09a4cbc3d5104
    static void imageCallback(void* context, AImageReader* reader)
    {
        AImage *image = nullptr;
        auto status = AImageReader_acquireNextImage(reader, &image);
        LOGA("imageCallback() %d", status);
        // Check status here ...

        // Try to process data without blocking the callback
        std::thread processor([=](){

            uint8_t *rPixel, *gPixel, *bPixel;//, *aPixel;
            int32_t rLen, gLen, bLen;//, aLen;
            AImage_getPlaneData(image, 0, &rPixel, &rLen);
            AImage_getPlaneData(image, 1, &gPixel, &gLen);
            AImage_getPlaneData(image, 2, &bPixel, &bLen);
            //AImage_getPlaneData(image, 3, &aPixel, &aLen);


            uint8_t * data = new uint8_t[rLen + gLen + bLen];
            memcpy(data, rPixel, rLen);
            memcpy(data + rLen, gPixel, gLen);
            memcpy(data + rLen + gLen, bPixel, bLen);
            //memcpy(data + rLen + gLen + bLen, aPixel, aLen);

            // Process data here
            cv::Mat gray_image = cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1, data);
            //cv::Mat gray_image;
            //cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
            cv::resize(gray_image, gray_image, cv::Size(), 0.5, 0.5);
            std::string my_str = "[ ";

            for(int i=0; i<gray_image.rows; i++)
            {
                my_str += "[";
                for(int j=0; j<gray_image.cols; j++)
                {
                    if(j == gray_image.cols -1)
                        my_str += std::to_string(gray_image.at<float>(i,j));
                    my_str += std::to_string(gray_image.at<float>(i,j)) + ", ";
                }
                if(i == gray_image.rows - 1)
                    my_str += "]";
                my_str += "],";
            }
            my_str += "]";
            LOGA("Converted image into cv mat = %s", my_str.c_str());
            AImage_delete(image);
        });
        processor.detach();
    }

    AImageReader* createJpegReader()
    {
        AImageReader* reader = nullptr;
        media_status_t status = AImageReader_new(IMAGE_WIDTH, IMAGE_HEIGHT, AIMAGE_FORMAT_YUV_420_888, 4, &reader);

        if (status != AMEDIA_OK)
        {
            LOGA("Error with image reader");
            return nullptr;
        }

        AImageReader_ImageListener listener{
                .context = this,
                .onImageAvailable = imageCallback,
        };

        AImageReader_setImageListener(reader, &listener);

        return reader;
    }

    ANativeWindow* createSurface(AImageReader* reader)
    {
        ANativeWindow *nativeWindow;
        AImageReader_getWindow(reader, &nativeWindow);

        return nativeWindow;
    }

    //Camera callbacks
    static void onDisconnected(void* context, ACameraDevice* device)
    {
        LOGA("onDisconnected");
    }

    static void onError(void* context, ACameraDevice* device, int error)
    {
        LOGA("error %d", error);
    }

    ACameraDevice_stateCallbacks cameraDeviceCallbacks = {
            .context = nullptr,
            .onDisconnected = onDisconnected,
            .onError = onError,
    };

    static void onSessionActive(void* context, ACameraCaptureSession *session)
    {
        LOGA("onSessionActive()");
    }

    static void onSessionReady(void* context, ACameraCaptureSession *session)
    {
        LOGA("onSessionReady()");
    }

    static void onSessionClosed(void* context, ACameraCaptureSession *session)
    {
        LOGA("onSessionClosed()");
    }

    ACameraCaptureSession_stateCallbacks sessionStateCallbacks {
            .context = nullptr,
            .onClosed = onSessionClosed,
            .onReady = onSessionReady,
            .onActive = onSessionActive,
    };

    static void onCaptureFailed(void* context, ACameraCaptureSession* session,
                         ACaptureRequest* request, ACameraCaptureFailure* failure)
    {
        LOGA("onCaptureFailed ");
    }

    static void onCaptureSequenceCompleted(void* context, ACameraCaptureSession* session,
                                    int sequenceId, int64_t frameNumber)
    {}

    static void onCaptureSequenceAborted(void* context, ACameraCaptureSession* session,
                                  int sequenceId)
    {}

    static void onCaptureCompleted (
            void* context, ACameraCaptureSession* session,
            ACaptureRequest* request, const ACameraMetadata* result)
    {
        LOGA("Capture completed");
    }

    ACameraCaptureSession_captureCallbacks captureCallbacks {
            .context = nullptr,
            .onCaptureStarted = nullptr,
            .onCaptureProgressed = nullptr,
            .onCaptureCompleted = onCaptureCompleted,
            .onCaptureFailed = onCaptureFailed,
            .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
            .onCaptureSequenceAborted = onCaptureSequenceAborted,
            .onCaptureBufferLost = nullptr,
    };


    std::string getBackFacingCamId(ACameraManager *cameraManager)
    {
        ACameraIdList *cameraIds = nullptr;
        ACameraManager_getCameraIdList(cameraManager, &cameraIds);

        std::string backId;

        LOGA("found camera count %d", cameraIds->numCameras);

        for (int i = 0; i < cameraIds->numCameras; ++i)
        {
            const char *id = cameraIds->cameraIds[i];

            ACameraMetadata *metadataObj;
            ACameraManager_getCameraCharacteristics(cameraManager, id, &metadataObj);
            LOGA("get camera characteristics %d", i);

            ACameraMetadata_const_entry lensInfo = {0};
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_FACING, &lensInfo);
            LOGA("lens info %d", i);

            auto facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
                    lensInfo.data.u8[0]);

            // Found a back-facing camera?
            if (facing == ACAMERA_LENS_FACING_BACK)
            {
                backId = id;
                break;
            }
        }
        LOGA("back camera id %s", backId.c_str());

        ACameraManager_deleteCameraIdList(cameraIds);
        return backId;
    }

    void initCam() {
        cameraManager = ACameraManager_create();
        auto id = getBackFacingCamId(cameraManager);
        ACameraManager_openCamera(cameraManager, id.c_str(), &cameraDeviceCallbacks, &cameraDevice);

        ACameraDevice_createCaptureRequest(cameraDevice, TEMPLATE_PREVIEW, &request);
        ACaptureSessionOutputContainer_create(&outputs);
        imageReader = createJpegReader();
        imageWindow = createSurface(imageReader);
        ANativeWindow_acquire(imageWindow);
        ACameraOutputTarget_create(imageWindow, &imageTarget);
        ACaptureRequest_addTarget(request, imageTarget);
        ACaptureSessionOutput_create(imageWindow, &imageOutput);
        ACaptureSessionOutputContainer_add(outputs, imageOutput);
        LOGA("CREATE CAPTURE SESSION");
        // Create the session
        camera_status_t status = ACameraDevice_createCaptureSession(cameraDevice, outputs, &sessionStateCallbacks, &captureSession);
        // Start capturing continuously
        status = ACameraCaptureSession_setRepeatingRequest(captureSession, &captureCallbacks, 1, &request, nullptr);
        LOGA("ACameraCaptureSession_setRepeatingRequest status = %d", status);

    }

    /// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
    /// _p_one_iteration() is called in a thread created by threadloop::start()
    void _p_one_iteration() override {
        LOGA("Android_cam started ..");
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
        if (!_m_clock->is_started()) {
            return;
        }
        double ts = std::chrono::system_clock::now().time_since_epoch().count();;
        ullong cam_time = static_cast<ullong>(ts * 1000000);
        if (!_m_first_cam_time) {
            _m_first_cam_time      = cam_time;
            _m_first_real_time_cam = _m_clock->now();
        }
        cv::Mat ir_left = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
        cv::Mat ir_right = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
        time_point cam_time_point{*_m_first_real_time_cam + std::chrono::nanoseconds(cam_time - *_m_first_cam_time)};
        LOGA("Writing to cam topic ..");
        _m_cam.put(_m_cam.allocate<cam_type>({cam_time_point, ir_left, ir_right}));
        LOGA("Done writing cam ..");
    }

private:
    const std::shared_ptr<switchboard>         sb;
    const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<cam_type>              _m_cam;
    std::mutex                                 mutex;

    std::optional<ullong>     _m_first_cam_time;
    std::optional<time_point> _m_first_real_time_cam;

    ACameraManager* cameraManager = nullptr;
    ACameraDevice* cameraDevice = nullptr;
    ANativeWindow* imageWindow = nullptr;
    ACameraOutputTarget* imageTarget = nullptr;
    AImageReader* imageReader = nullptr;
    ACaptureSessionOutput* imageOutput = nullptr;
    ACaptureRequest* request = nullptr;
    //ACaptureSessionOutput* output = nullptr;
    ACaptureSessionOutputContainer* outputs = nullptr;
    ACameraCaptureSession* captureSession = nullptr;
    static const int IMAGE_WIDTH = 640;
    static const int IMAGE_HEIGHT = 480;
};

PLUGIN_MAIN(android_cam);

