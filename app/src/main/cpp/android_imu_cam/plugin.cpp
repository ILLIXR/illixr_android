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
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraDevice.h>
#include <media/NdkImageReader.h>
#include <android/native_window_jni.h>
#include <android/sensor.h>
#include <Eigen/Core>
#include <fstream>
#include <filesystem>

#define LOGA(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_cam", __VA_ARGS__))
#define POLL_RATE_USEC 5000

using namespace ILLIXR;

struct android_imu_struct
{
    ASensorManager *sensor_manager;
    const ASensor *accelerometer;
    const ASensor *gyroscope;
    const ASensor* gravity;
    ASensorEventQueue *event_queue;
};
static std::mutex _m_lock;
static Eigen::Vector3f gyro;
static Eigen::Vector3f accel;
static Eigen::Vector3f grav;
static cv::Mat last_image;
static bool img_ready = false;
static std::mutex mtx;
//static double current_ts = 0;
std::ofstream myfile;
std::ofstream camfile;
bool once = false;
int counter = 0;
static std::deque<std::pair<Eigen::Vector3f, Eigen::Vector3f>> avg_filter;

class android_imu_cam : public threadloop {
public:
    android_imu_cam(std::string name_, phonebook* pb_)
            : threadloop{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , _m_clock{pb->lookup_impl<RelativeClock>()}
            , _m_imu_cam{sb->get_writer<imu_cam_type>("imu_cam")}{
        LOGA("IMU CAM CONSTRUCTOR");
        initCam();
        //std::filesystem::create_directories("/sdcard/Android/data/com.example.native_activity/cam0/");
//        remove("/sdcard/Android/data/com.example.native_activity/cam0/*.png");
        myfile.open ("/sdcard/Android/data/com.example.native_activity/imu0.csv");
        camfile.open("/sdcard/Android/data/com.example.native_activity/data.csv");
        struct android_imu_struct *imu = new android_imu_struct();
        std::thread (android_run_thread, imu).detach();
    }

    virtual ~android_imu_cam() override {
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
        myfile.close();
        camfile.close();
    }

    static std::pair<Eigen::Vector3f, Eigen::Vector3f> moving_average() {
        std::pair<Eigen::Vector3f, Eigen::Vector3f> avg;
        if(avg_filter.size() == 0)
            return avg;
        for(int i = 0; i < avg_filter.size() ; ++i) {
            avg.first += avg_filter[i].first;
            avg.second += avg_filter[i].second;
        }
        avg.first = avg.first/avg_filter.size();
        avg.second = avg.second/avg_filter.size();
        return avg;
    }

    static int
    android_sensor_callback(int fd, int events, void *data)
    {
        struct android_imu_struct *d = (struct android_imu_struct *)data;
        //LOGA("Android sensor callback");
        if (d->accelerometer == NULL || d->gyroscope == NULL)
            return 1;

        ASensorEvent event;
        while (ASensorEventQueue_getEvents(d->event_queue, &event, 1) > 0) {
            auto start = std::chrono::high_resolution_clock::now();
            _m_lock.lock();
            switch (event.type) {
//                case ASENSOR_TYPE_GRAVITY: {
//                    grav[0] = -event.acceleration.z;
//                    grav[1] = -event.acceleration.x;
//                    grav[2] = event.acceleration.y;
//                    LOGA("Gravity sensor values %f %f %f", grav[0], grav[1], grav[2]);
//                    break;
//                }

                case ASENSOR_TYPE_ACCELEROMETER: {
//                    accel[0] = event.acceleration.y;
//                    accel[1] = -event.acceleration.x;
//                    accel[2] = event.acceleration.z;

//matches zed
//                    accel[0] = -event.acceleration.z;//  - grav[0];;
//                    accel[1] = -event.acceleration.x;// - grav[1];
//                    accel[2] = event.acceleration.y;// - grav[2] - 9.8;

                    accel[0] = event.acceleration.x;
                    accel[1] = event.acceleration.y;
                    accel[2] = event.acceleration.z;

//                    accel[0] = 1;
//                    accel[1] = 0;
//                    accel[2] = 0;
                    break;
                }
                case ASENSOR_TYPE_GYROSCOPE: {
//                    gyro[0] = -event.data[1];
//                    gyro[1] = event.data[0];
//                    gyro[2] = event.data[2];

//matches zed
//                    gyro[0] = -event.data[2];
//                    gyro[1] = -event.data[0];
//                    gyro[2] = event.data[1];

                    gyro[0] = event.data[0];
                    gyro[1] = event.data[1];
                    gyro[2] = event.data[2];
//                    gyro[0] = 0;
//                    gyro[1] = 0;
//                    gyro[2] = 0;
//                    timestamp = event.timestamp;
//                    avg_filter.push_back({accel,gyro});
//                    std::pair<Eigen::Vector3f, Eigen::Vector3f> avg_imu = moving_average();
//                    accel = avg_imu.first;
//                    gyro = avg_imu.second;
//                    if(avg_filter.size() >= 10) {
//                        avg_filter.pop_front();
//                    }
                    //uint64_t time_s = std::chrono::system_clock::now().time_since_epoch().count();
                    //LOGA("IMU Values : accel %f %f %f %f %f %f", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);

                }
                default: ;
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration =  std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            LOGA("duration: %f", duration2double(duration));
            _m_lock.unlock();

        }

        return 1;
    }

    static void *
    android_run_thread(void *ptr)
    {
        //myfile.open ("/sdcard/Android/data/com.example.native_activity/imu0.csv");
        struct android_imu_struct *d = (struct android_imu_struct *)ptr;
        const int32_t poll_rate_usec = POLL_RATE_USEC;

#if __ANDROID_API__ >= 26
        d->sensor_manager = ASensorManager_getInstanceForPackage("ILLIXR_ANDROID_IMU");
#else
        d->sensor_manager = ASensorManager_getInstance();
#endif

        d->accelerometer = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_ACCELEROMETER);
        d->gyroscope = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_GYROSCOPE);
//        d->gravity = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_GRAVITY);


        ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

        d->event_queue = ASensorManager_createEventQueue(d->sensor_manager, looper, ALOOPER_POLL_CALLBACK,
                                                         android_sensor_callback, (void *)d);

        // Start sensors in case this was not done already.
        if (d->accelerometer != NULL) {
            ASensorEventQueue_enableSensor(d->event_queue, d->accelerometer);
            ASensorEventQueue_setEventRate(d->event_queue, d->accelerometer, poll_rate_usec);
        }
        if (d->gyroscope != NULL) {
            ASensorEventQueue_enableSensor(d->event_queue, d->gyroscope);
            ASensorEventQueue_setEventRate(d->event_queue, d->gyroscope, poll_rate_usec);
        }
//        if (d->gravity != NULL) {
//            ASensorEventQueue_enableSensor(d->event_queue, d->gravity);
//            ASensorEventQueue_setEventRate(d->event_queue, d->gravity, poll_rate_usec);
//        }
        int ret = 0;
        while (ret != ALOOPER_POLL_ERROR) {
            ret = ALooper_pollAll(0, NULL, NULL, NULL);
        }
        //myfile.close();

        return NULL;
    }

    //Reference: https://github.com/sixo/native-camera/tree/93b05aec6d05604a314dc822b6b09a4cbc3d5104
     static void imageCallback(void* context, AImageReader* reader)
    {
        AImage *image = nullptr;
        AImageReader_acquireNextImage(reader, &image);
        //LOGA("Image callback");
        if(image == nullptr)
        {
            LOGA("IMage is null!");
            return;
        }
        auto start = std::chrono::high_resolution_clock::now();


        // Try to process data without blocking the callback
        //std::thread processor([=](){
            uint8_t *rPixel;
            int32_t rLen;
            int32_t yPixelStride, yRowStride;
            AImage_getPlanePixelStride(image, 0, &yPixelStride);
            AImage_getPlaneRowStride(image, 0, &yRowStride);
            AImage_getPlaneData(image, 0, &rPixel, &rLen);
            uint8_t * data = new uint8_t[rLen];

            if (yPixelStride == 1) {
                for (int y = 0; y < IMAGE_HEIGHT; y++)
                    memcpy(data + y*IMAGE_WIDTH, rPixel + y*yRowStride, IMAGE_WIDTH);
            }
            cv::Mat rawData( IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1, (uint8_t *)data);
            mtx.lock();
            last_image = rawData;
            mtx.unlock();
            img_ready = true;
            AImage_delete(image);
//        });
//        processor.detach();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration =  std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        LOGA("duration: %f", duration2double(duration));
    }


    AImageReader* createJpegReader()
    {
        AImageReader* reader = nullptr;
        media_status_t status =
        AImageReader_new(IMAGE_WIDTH, IMAGE_HEIGHT, AIMAGE_FORMAT_YUV_420_888 , 10, &reader);

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
    {}

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

            ACameraMetadata_const_entry pose_reference = {0};
            ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_POSE_REFERENCE, &pose_reference);

            auto pose_ref = static_cast<acamera_metadata_enum_acamera_lens_pose_reference>(
                    pose_reference.data.u8[0]);

            LOGA("Pose reference is = %d and facing = %d", pose_ref, facing);
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

                ACameraMetadata_const_entry distortion;
                ACameraMetadata_getConstEntry(metadataObj, ACAMERA_LENS_DISTORTION, &distortion);
                for(int x = 0; x < distortion.count ; ++x) {
                    LOGA("distortion  = %d : %f : index %d", distortion.count, distortion.data.f[x], x);
                }

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
        LOGA("INIT CAM");
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
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
        if(!img_ready)
            return;
        if (!_m_clock->is_started()) {
            return;
        }
        auto start = std::chrono::high_resolution_clock::now();

        double ts = std::chrono::system_clock::now().time_since_epoch().count();//current_ts;
        ullong cam_time = static_cast<ullong>(ts * 1000);
        if (!_m_first_cam_time) {
            _m_first_cam_time      = cam_time;
            _m_first_real_time_cam = _m_clock->now();
        }
        _m_lock.lock();
        Eigen::Vector3f cur_gyro = gyro;
        Eigen::Vector3f cur_accel = accel;
        counter++;
        myfile << std::to_string((uint64_t)ts*1000) + "," + std::to_string(gyro[0]) + "," + std::to_string(gyro[1]) + "," +
                  std::to_string(gyro[2]) + "," + std::to_string(accel[0]) + ","  + std::to_string(accel[1]) + "," + std::to_string(accel[2])  + "\n";
        _m_lock.unlock();

        std::optional<cv::Mat>  ir_left = std::nullopt;
        std::optional<cv::Mat>  ir_right = std::nullopt;
        if(counter%10 == 0) {
            mtx.lock();
            ir_left = std::make_optional<cv::Mat>(last_image);
            ir_right = std::make_optional<cv::Mat>(last_image);
            uint64_t name = (uint64_t)ts*1000;
//            bool check = cv::imwrite(
//                    "/sdcard/Android/data/com.example.native_activity/cam0/"+ std::to_string(name)+".png", last_image);
//            LOGA("ANDROID CAM CHECK %d", check);
            camfile << std::to_string((uint64_t)ts*1000) << "," << std::to_string(name) + ".png" <<std::endl;
            mtx.unlock();
        }

        time_point cam_time_point{*_m_first_real_time_cam + std::chrono::nanoseconds(cam_time - *_m_first_cam_time)};
        //LOGA("TIME = %lf and cam_time %llu",duration2double(std::chrono::nanoseconds(cam_time - *_m_first_cam_time)), cam_time);
//        _m_cam.put(_m_cam.allocate<cam_type>({cam_time_point, ir_left, ir_right}));
        _m_imu_cam.put(_m_imu_cam.allocate<imu_cam_type>(
                            imu_cam_type{time_point{cam_time_point},
                                         cur_gyro.cast<float>(),
                                         cur_accel.cast<float>(), ir_left, ir_right}));
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration =  std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        LOGA("duration: %f", duration2double(duration));
    }

private:
    const std::shared_ptr<switchboard>         sb;
    const std::shared_ptr<const RelativeClock> _m_clock;
    //switchboard::writer<cam_type>              _m_cam;
    switchboard::writer<imu_cam_type>              _m_imu_cam;

    std::optional<ullong>     _m_first_cam_time;
    std::optional<time_point> _m_first_real_time_cam;

    ACameraManager* cameraManager = nullptr;
    ACameraDevice* cameraDevice = nullptr;
    ANativeWindow* imageWindow = nullptr;
    ACameraOutputTarget* imageTarget = nullptr;
    AImageReader* imageReader = nullptr;
    ACaptureSessionOutput* imageOutput = nullptr;
    ACaptureRequest* request = nullptr;
    ACaptureSessionOutputContainer* outputs = nullptr;
    ACameraCaptureSession* captureSession = nullptr;
    static const int IMAGE_WIDTH = 752;
    static const int IMAGE_HEIGHT = 480;
};

PLUGIN_MAIN(android_imu_cam);

