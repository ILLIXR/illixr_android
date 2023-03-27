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
#include <fstream>
#include <android/log.h>
//#include <android/imagedecoder.h>
//#include <android/bitmap.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraDevice.h>
#include <media/NdkImageReader.h>
#include <android/native_window_jni.h>

#define LOGA(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_cam", __VA_ARGS__))

using namespace ILLIXR;

static cv::Mat last_image;
static bool img_ready = false;
static std::mutex mtx;
bool once = false;

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
        //auto status =
        AImageReader_acquireNextImage(reader, &image);
        if(image == nullptr)
        {
            LOGA("IMage is null!");
            return;
        }
        //LOGA("imageCallback()");
        // Check status here ...

        // Try to process data without blocking the callback
        std::thread processor([=](){

            uint8_t *rPixel;//, *gPixel, *bPixel;//, *aPixel;
            int32_t rLen;//, gLen, bLen;//, aLen;
            int32_t yPixelStride, yRowStride;
            AImage_getPlanePixelStride(image, 0, &yPixelStride);
            AImage_getPlaneRowStride(image, 0, &yRowStride);
            AImage_getPlaneData(image, 0, &rPixel, &rLen);
            //LOGA("Pixel stride = %d and row stride = %d", yPixelStride, yRowStride);
            uint8_t * data = new uint8_t[rLen];//+ gLen + bLen

            if (yPixelStride == 1) {
                for (int y = 0; y < IMAGE_HEIGHT; y++)
                    memcpy(data + y*IMAGE_WIDTH, rPixel + y*yRowStride, IMAGE_WIDTH);
            }
//            AImage_getPlaneData(image, 1, &gPixel, &gLen);
//            AImage_getPlaneData(image, 2, &bPixel, &bLen);
            //AImage_getPlaneData(image, 3, &aPixel, &aLen);

            //memcpy(data, rPixel, rLen);
            //memcpy(data + rLen, gPixel, gLen);
            //memcpy(data + rLen + gLen, bPixel, bLen);
            //memcpy(data + rLen + gLen + bLen, aPixel, aLen);
            cv::Mat rawData( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, (uint8_t *)data);
            if(!once) {
                bool check = cv::imwrite(
                        "/sdcard/Android/data/com.example.native_activity/my_img.png", rawData);
                LOGA("ANDROID CAM CHECK %d", check);
                once = true;

//
//                std::ofstream fout("/sdcard/Android/data/com.example.native_activity/img.txt");
//
//                if (!fout) {
//                    LOGA("fout fail");
//                }
//
//                for (int i = 0; i < rawData.rows; i++) {
//                    for (int j = 0; j < rawData.cols; j++) {
//                        fout << rawData.at<uint8_t>(i, j) << "\t";
//                    }
//                    fout << "\n";
//                    LOGA("READING");
//                }
//
//                fout.close();
            }
            mtx.lock();
            last_image = rawData;
            mtx.unlock();
            img_ready = true;
           // is_ready = true;

//            cv::Mat decodedImage = cv::imdecode( rawData, cv::IMREAD_GRAYSCALE/*, flags */ );
//            if ( decodedImage.data == NULL )
//            {
//                LOGA("Jpeg image decoding failed");
//            }
//            // Process data here
//            cv::Mat gray_image = cv::Mat(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1, data);

            //cv::Mat gray_image;
            //cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
 //           cv::resize(gray_image, gray_image, cv::Size(), 0.5, 0.5);
 //            if(once == false) {
//                 std::string dt = "";
//                 int each = 120;
//                 for (int i = 0; i < rLen; ++i) {
//                     if (each == 0) {
//                         LOGA("Raw buffer = %s", dt.c_str());
//                         each = 120;
//                         dt = "";
//                     }
//
//                     dt = dt + std::to_string(data[i]) + " ";
//                     each--;
//                 }
//                 LOGA("Raw buffer = %s", dt.c_str());
//                 once = true;
//             }
//                 std::string my_str = "[ ";
//
//                 for (int i = 0; i < rawData.rows; i++) {
//                     my_str = "[";
//                     for (int j = 0; j < rawData.cols; j++) {
//                         if (j == rawData.cols - 1)
//                             my_str += std::to_string(rawData.at<uint8_t>(i, j));
//                         else
//                             my_str += std::to_string(rawData.at<uint8_t>(i, j)) + ", ";
//                     }
//                     if (i == rawData.rows - 1) {
//                         my_str += "]";
//                         LOGA("Image printed = %s", my_str.c_str());
//                     } else {
//                         my_str += "],";
//                         LOGA("Image printed = %s", my_str.c_str());
//                     }
//                 }
//                 my_str += "]";
//                 once = true;
//             }

           // LOGA("Converted image into cv mat");
            AImage_delete(image);
        });
        processor.detach();
    }


    AImageReader* createJpegReader()
    {
        AImageReader* reader = nullptr;
        //AIMAGE_FORMAT_JPEG
        //AIMAGE_FORMAT_RGB_888
        media_status_t status =
        AImageReader_new(IMAGE_WIDTH, IMAGE_HEIGHT, AIMAGE_FORMAT_YUV_420_888 , 4, &reader);

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
        //LOGA("Capture completed");
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
        bool camera_found = false;
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

//            ACameraMetadata_const_entry pose_reference = {0};
//            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_REFERENCE, &pose_reference);
//
//            auto pose_ref = static_cast<acamera_metadata_enum_acamera_lens_pose_reference>(
//                    pose_reference.data.u8[0]);

//            LOGA("Pose reference is = %d and facing = %d", pose_ref, facing);
            // Found a back-facing camera?
            if (facing == ACAMERA_LENS_FACING_BACK)
            {
                backId = id;
                camera_found = true;
            }

            if(camera_found) {
                ACameraMetadata_const_entry intrinsics;
                ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_INTRINSIC_CALIBRATION,
                                              &intrinsics);
                for (int x = 0; x < intrinsics.count; ++x) {
                    LOGA("Intrinsics total = %d : %f : index %d", intrinsics.count,
                         intrinsics.data.f[x], x);
                }

//                ACameraMetadata_const_entry distortion;
//                ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_DISTORTION, &distortion);
//                for(int x = 0; x < distortion.count ; ++x) {
//                    LOGA("distortion  = %d : %f : index %d", distortion.count, distortion.data.f[x], x);
//                }

                ACameraMetadata_const_entry rotation;
                ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_ROTATION, &rotation);
                for (int x = 0; x < rotation.count; ++x) {
                    LOGA("rotation total = %d : %f : index %d", rotation.count, rotation.data.f[x], x);
                }

                ACameraMetadata_const_entry translation;
                ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_TRANSLATION, &translation);
                for (int x = 0; x < translation.count; ++x) {
                    LOGA("translation total = %d : %f : index %d", translation.count, translation.data.f[x], x);
                }
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
        // Create the session
        ACameraDevice_createCaptureSession(cameraDevice, outputs, &sessionStateCallbacks, &captureSession);
        camera_status_t status = ACameraCaptureSession_setRepeatingRequest(captureSession, &captureCallbacks, 1, &request, nullptr);
        LOGA("ACameraCaptureSession_setRepeatingRequest status = %d", status);

    }

    /// For `threadloop` style plugins, do not override the start() method unless you know what you're doing!
    /// _p_one_iteration() is called in a thread created by threadloop::start()
    void _p_one_iteration() override {
        //LOGA("Android_cam started ..");
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
        if(!img_ready)
            return;
        if (!_m_clock->is_started()) {
            return;
        }
       // camera_status_t status = ACameraCaptureSession_capture(captureSession, &captureCallbacks, 1, &request, nullptr);
       // LOGA("ACameraCaptureSession_setRepeatingRequest status = %d", status);

        double ts = std::chrono::system_clock::now().time_since_epoch().count();;
        ullong cam_time = static_cast<ullong>(ts * 1000);
        if (!_m_first_cam_time) {
            _m_first_cam_time      = cam_time;
            _m_first_real_time_cam = _m_clock->now();
        }

//        while(is_ready == false) {
//            ;
//        }
        mtx.lock();
        cv::Mat ir_left = last_image;//cv::Mat::zeros(cv::Size(100, 100), CV_8UC1);
        cv::Mat ir_right = last_image;//cv::Mat::zeros(cv::Size(100, 100), CV_8UC1);
        mtx.unlock();
        //is_ready = false;
//        cv::cvtColor(ir_left,ir_left,cv::COLOR_BGR2GRAY);
//        cv::cvtColor(ir_right,ir_right,cv::COLOR_BGR2GRAY);

        time_point cam_time_point{*_m_first_real_time_cam + std::chrono::nanoseconds(cam_time - *_m_first_cam_time)};
        //LOGA("Writing to cam topic .. width = %d, height =  %d", ir_left.cols , ir_left.rows);
        LOGA("TIME = %lf and cam_time %llu",duration2double(std::chrono::nanoseconds(cam_time - *_m_first_cam_time)), cam_time);
        _m_cam.put(_m_cam.allocate<cam_type>({cam_time_point, ir_left, ir_right}));
        //LOGA("Done writing cam ..");
    }

private:
    const std::shared_ptr<switchboard>         sb;
    const std::shared_ptr<const RelativeClock> _m_clock;
    switchboard::writer<cam_type>              _m_cam;



    std::optional<ullong>     _m_first_cam_time;
    std::optional<time_point> _m_first_real_time_cam;

    ACameraManager* cameraManager = nullptr;
    ACameraDevice* cameraDevice = nullptr;
    ANativeWindow* imageWindow = nullptr;
    ACameraOutputTarget* imageTarget = nullptr;
    AImageReader* imageReader = nullptr;
    ACaptureSessionOutput* imageOutput = nullptr;
    ACaptureRequest* request = nullptr;
//    ACaptureSessionOutput* output = nullptr;
    ACaptureSessionOutputContainer* outputs = nullptr;
    ACameraCaptureSession* captureSession = nullptr;
    static const int IMAGE_WIDTH = 752;
    static const int IMAGE_HEIGHT = 480;

};

PLUGIN_MAIN(android_cam);

