#include "plugin.hpp"

#include "illixr/data_format/misc.hpp"

#include <android/sensor.h>
#include <android/log.h>
#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <fstream>

#define POLL_RATE_USEC 5000 //(1000L / 60) * 1000
#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "android_imu", __VA_ARGS__))

using namespace ILLIXR;

/// Inherit from plugin if you don't need the threadloop
/// Inherit from threadloop to provide a new thread to perform the task
struct android_imu_struct {
    ASensorManager* sensor_manager;
    const ASensor* accelerometer;
    const ASensor* gyroscope;
    ASensorEventQueue* event_queue;
};
static std::mutex _m_lock;
static std::optional<ullong> _m_first_imu_time;
static std::optional<time_point> _m_first_real_time_imu;
static Eigen::Vector3f gyro;
static Eigen::Vector3f accel;
static unsigned long long timestamp;
static unsigned long long last_timestamp = 0;
std::ofstream myfile;

[[maybe_unused]] android_imu::android_imu(const std::string& name_, phonebook* pb_)
        : threadloop{name_, pb_}
        , switchboard_{pb->lookup_impl<switchboard>()}
        , clock_{pb->lookup_impl<RelativeClock>()}
        , imu_{switchboard_->get_writer<data_format::imu_type>("imu")} {
    struct android_imu_struct* imu = new android_imu_struct();
    std::thread(android_run_thread, imu).detach();
}

android_imu::~android_imu() {
    std::cout << "Deconstructing basic_plugin." << std::endl;
}

int android_imu::android_sensor_callback(int fd, int events, void* data) {
    struct android_imu_struct* d = (struct android_imu_struct*) data;
    ANDROID_LOG("Android sensor callback");
    if (d->accelerometer == NULL || d->gyroscope == NULL)
        return 1;

    ASensorEvent event;
    while (ASensorEventQueue_getEvents(d->event_queue, &event, 1) > 0) {
        _m_lock.lock();
        switch (event.type) {

            case ASENSOR_TYPE_ACCELEROMETER: {
//                    accel[0] = event.acceleration.y;
//                    accel[1] = -event.acceleration.x;
//                    accel[2] = event.acceleration.z;

                accel[0] = event.acceleration.x;
                accel[1] = event.acceleration.y;
                accel[2] = event.acceleration.z;

//                    accel[0] = event.acceleration.z;
//                    accel[1] = -event.acceleration.x;
//                    accel[2] = event.acceleration.y;

//                    accel[0] = 1;
//                    accel[1] = 0;
//                    accel[2] = 0;
                break;
            }
            case ASENSOR_TYPE_GYROSCOPE: {
//                    gyro[0] = -event.data[1];
//                    gyro[1] = event.data[0];
//                    gyro[2] = event.data[2];

                gyro[0] = event.data[0];
                gyro[1] = event.data[1];
                gyro[2] = event.data[2];

//                    gyro[0] = event.data[2];
//                    gyro[1] = -event.data[0];
//                    gyro[2] = event.data[1];
                ;
//                    gyro[0] = 0;
//                    gyro[1] = 0;
//                    gyro[2] = 0;
//                    timestamp = event.timestamp;

                uint64_t time_s = std::chrono::system_clock::now().time_since_epoch().count();
                timestamp = time_s;
                ANDROID_LOG("IMU Values : accel %f %f %f %f %f %f", accel[0], accel[1], accel[2],
                            gyro[0], gyro[1], gyro[2]);
//                    myfile << std::to_string(time_s) + "," + std::to_string(gyro[0]) + "," + std::to_string(gyro[1]) + "," +
//                              std::to_string(gyro[2]) + "," + std::to_string(accel[0]) + ","  + std::to_string(accel[1]) + "," + std::to_string(accel[2]) + "," + "\n";
            }
            default:;
        }
        _m_lock.unlock();

    }

    return 1;
}

void* android_imu::android_run_thread(void* ptr) {
    myfile.open("/sdcard/Android/data/com.example.native_activity/imu0.csv");
    struct android_imu_struct* d = (struct android_imu_struct*) ptr;
    const int32_t poll_rate_usec = POLL_RATE_USEC;

#if __ANDROID_API__ >= 26
    d->sensor_manager = ASensorManager_getInstanceForPackage("ILLIXR_ANDROID_IMU");
#else
    d->sensor_manager = ASensorManager_getInstance();
#endif

    d->accelerometer = ASensorManager_getDefaultSensor(d->sensor_manager,
                                                       ASENSOR_TYPE_ACCELEROMETER);
    d->gyroscope = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_GYROSCOPE);

    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

    d->event_queue = ASensorManager_createEventQueue(d->sensor_manager, looper,
                                                     ALOOPER_POLL_CALLBACK,
                                                     android_sensor_callback, (void*) d);

    // Start sensors in case this was not done already.
    if (d->accelerometer != NULL) {
        ASensorEventQueue_enableSensor(d->event_queue, d->accelerometer);
        ASensorEventQueue_setEventRate(d->event_queue, d->accelerometer, poll_rate_usec);
    }
    if (d->gyroscope != NULL) {
        ASensorEventQueue_enableSensor(d->event_queue, d->gyroscope);
        ASensorEventQueue_setEventRate(d->event_queue, d->gyroscope, poll_rate_usec);
    }

    int ret = 0;
    while (ret != ALOOPER_POLL_ERROR) {
        ret = ALooper_pollAll(0, NULL, NULL, NULL);
    }
    myfile.close();

    return NULL;
}

void android_imu::_p_one_iteration() {

    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    if (!clock_->is_started()) {
        return;
    }
    _m_lock.lock();
    if (last_timestamp == timestamp) {
        _m_lock.unlock();
        return;
    }
    last_timestamp = timestamp;

    ullong imu_time = static_cast<ullong>(timestamp * 1000);
    if (!_m_first_imu_time) {
        _m_first_imu_time = imu_time;
        _m_first_real_time_imu = clock_->now();
    }
    time_point imu_time_point{
            *_m_first_real_time_imu + std::chrono::nanoseconds(imu_time - *_m_first_imu_time)};
    ANDROID_LOG("ANDROID_IMU WRITING %f",
                duration2double(std::chrono::nanoseconds(imu_time - *_m_first_imu_time)));
    imu_.put(
            imu_.allocate<data_format::imu_type>({imu_time_point, accel.cast<double>(), gyro.cast<double>()}));
    _m_lock.unlock();
}


/// This line makes the plugin importable by Spindle
PLUGIN_MAIN(android_imu)
