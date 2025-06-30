#include "plugin.hpp"


#include <boost/smart_ptr/make_shared.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h> // Used if IMU combined is off.
#include <gtsam/navigation/ImuFactor.h>
#include <iomanip>
#include <thread>

using namespace ILLIXR;
// IMU sample time to live in seconds
constexpr duration IMU_TTL{std::chrono::seconds{5}};

using ImuBias = gtsam::imuBias::ConstantBias;

[[maybe_unused]] gtsam_integrator::gtsam_integrator(const std::string& name_, phonebook* pb_)
        : plugin{name_, pb_}
        , switchboard_{pb_->lookup_impl<switchboard>()}
        , clock_{pb_->lookup_impl<relative_clock>()}
        , imu_integrator_input_{switchboard_->get_reader<data_format::imu_integrator_input>("imu_integrator_input")}
        , imu_raw_{switchboard_->get_writer<data_format::imu_raw_type>("imu_raw")} {
    switchboard_->schedule<data_format::imu_type>(id, "imu",
                                                  [&](switchboard::ptr<const data_format::imu_type> datum,
                                            size_t) {
                                            callback(datum);
                                        });
}

void gtsam_integrator::callback(switchboard::ptr<const data_format::imu_type> datum) {
    imu_vec_.emplace_back(datum->time, datum->angular_v, datum->linear_a);

    clean_imu_vec(datum->time);
    propagate_imu_values(datum->time);

    RAC_ERRNO_MSG("gtsam_integrator");
}


/**
 * @brief Wrapper object protecting the lifetime of IMU integration inputs and biases
 */

gtsam_integrator::PimObject::PimObject(const imu_int_t& imu_int_input)
        : _imu_bias{imu_int_input.bias_acc, imu_int_input.bias_gyro}, _pim{nullptr} {
    pim_t::Params _params{imu_int_input.params.n_gravity};
    _params.setGyroscopeCovariance(
            std::pow(imu_int_input.params.gyro_noise, 2.0) * Eigen::Matrix3d::Identity());
    _params.setAccelerometerCovariance(
            std::pow(imu_int_input.params.acc_noise, 2.0) * Eigen::Matrix3d::Identity());
    _params.setIntegrationCovariance(std::pow(imu_int_input.params.imu_integration_sigma, 2.0) *
                                     Eigen::Matrix3d::Identity());
    _params.setBiasAccCovariance(
            std::pow(imu_int_input.params.acc_walk, 2.0) * Eigen::Matrix3d::Identity());
    _params.setBiasOmegaCovariance(
            std::pow(imu_int_input.params.gyro_walk, 2.0) * Eigen::Matrix3d::Identity());

    pim_ = new pim_t{boost::make_shared<pim_t::Params>(std::move(_params)), imu_bias_};
    resetIntegrationAndSetBias(imu_int_input);
}

gtsam_integrator::PimObject::~PimObject() {
    assert(pim_ != nullptr && "_pim should not be null");

    /// Note: Deliberately leak _pim => Removes SEGV read during destruction
    /// delete _pim;
};

void
gtsam_integrator::PimObject::resetIntegrationAndSetBias(const imu_int_t& imu_int_input) noexcept {
    assert(pim_ != nullptr && "_pim should not be null");

    imu_bias_ = bias_t{imu_int_input.biasAcc, imu_int_input.biasGyro};
    pim_->resetIntegrationAndSetBias(imu_bias_);

    navstate_lkf_ =
            nav_t{gtsam::Pose3{gtsam::Rot3{imu_int_input.quat}, imu_int_input.position},
                  imu_int_input.velocity};
}

void gtsam_integrator::PimObject::integrateMeasurement(const imu_t& imu_input,
                                                       const imu_t& imu_input_next) noexcept {
    assert(pim_ != nullptr && "_pim shuold not be null");

    const gtsam::Vector3 measured_acc{imu_input.linear_a};
    const gtsam::Vector3 measured_omega{imu_input.angular_v};

    duration delta_t = imu_input_next.time - imu_input.time;

    pim_->integrateMeasurement(measured_acc, measured_omega, duration_to_double(delta_t));
}

bias_t gtsam_integrator::PimObject::biasHat() const noexcept {
    assert(pim_ != nullptr && "_pim shuold not be null");
    return pim_->biasHat();
}

nav_t gtsam_integrator::PimObject::predict() const noexcept {
    assert(pim_ != nullptr && "_pim should not be null");
    return pim_->predict(navstate_lkf_, imu_bias_);
}


// Remove IMU values older than 'IMU_TTL' from the imu buffer
void gtsam_integrator::clean_imu_vec(time_point timestamp) {
    auto imu_iterator = imu_vec_.begin();

    // Since the vector is ordered oldest to latest, keep deleting until you
    // hit a value less than 'IMU_TTL' seconds old
    while (imu_iterator != imu_vec_.end()) {
        if (timestamp - imu_iterator->time < IMU_TTL) {
            break;
        }

        imu_iterator = imu_vec_.erase(imu_iterator);
    }
}

// Timestamp we are propagating the biases to (new IMU reading time)
void gtsam_integrator::propagate_imu_values(time_point real_time) {
    auto input_values = imu_integrator_input_.get_ro_nullable();
    if (input_values == nullptr) {
        return;
    }

#ifndef NDEBUG
    if (input_values->last_cam_integration_time > last_cam_time_) {
        std::cout << "New slow pose has arrived!\n";
        last_cam_time_ = input_values->last_cam_integration_time;
    }
#endif

    if (_pim_obj == nullptr) {
        /// We don't have a PimObject -> make and set given the current input
        _pim_obj = std::make_unique<PimObject>(*input_values);

        /// Setting 'last_imu_offset' here to stay consistent with previous integrator version.
        /// TODO: Should be set and tested at the end of this function to avoid staleness from VIO.
        last_imu_offset_ = input_values->t_offset;
    } else {
        /// We already have a PimObject -> set the values given the current input
        _pim_obj->resetIntegrationAndSetBias(*input_values);
    }

    assert(_pim_obj != nullptr && "_pim_obj should not be null");

    time_point time_begin = input_values->last_cam_integration_time + last_imu_offset_;
    time_point time_end = input_values->t_offset + real_time;

    const std::vector<data_format::imu_type> prop_data = select_imu_readings(imu_vec_, time_begin,
                                                                             time_end);

    /// Need to integrate over a sliding window of 2 imu_type values.
    /// If the container of data is smaller than 2 elements, return early.
    if (prop_data.size() < 2) {
        return;
    }

    ImuBias prev_bias = _pim_obj->biasHat();
    ImuBias bias = _pim_obj->biasHat();

#ifndef NDEBUG
    std::cout << "Integrating over " << prop_data.size() << " IMU samples\n";
#endif

    for (std::size_t i = 0; i < prop_data.size() - 1; i++) {
        _pim_obj->integrateMeasurement(prop_data[i], prop_data[i + 1]);

        prev_bias = bias;
        bias = _pim_obj->biasHat();
    }

    gtsam::NavState navstate_k = _pim_obj->predict();
    gtsam::Pose3 out_pose = navstate_k.pose();

#ifndef NDEBUG
    std::cout << "Base Position (x, y, z) = " << input_values->position(0) << ", "
              << input_values->position(1) << ", "
              << input_values->position(2) << std::endl;

    std::cout << "New  Position (x, y, z) = " << out_pose.x() << ", " << out_pose.y() << ", "
              << out_pose.z() << std::endl;
#endif

    imu_raw_.put(imu_raw_.allocate<data_format::imu_raw_type>(
            data_format::imu_raw_type{prev_bias.gyroscope(), prev_bias.accelerometer(),
                                      bias.gyroscope(), bias.accelerometer(),
                                      out_pose.translation(),             /// Position
                                      navstate_k.velocity(),              /// Velocity
                                      out_pose.rotation().toQuaternion(), /// Eigen Quat
                                      real_time}));
}

// Select IMU readings based on timestamp similar to how OpenVINS selects IMU values to propagate
std::vector<data_format::imu_type>
gtsam_integrator::select_imu_readings(const std::vector<data_format::imu_type>& imu_data,
                                      const time_point time_begin,
                                      const time_point time_end) {
    std::vector<data_format::imu_type> prop_data;
    if (imu_data.size() < 2) {
        return prop_data;
    }

    for (std::size_t i = 0; i < imu_data.size() - 1; i++) {
        // If time_begin comes inbetween two IMUs (A and B), interpolate A forward to time_begin
        if (imu_data[i + 1].time > time_begin && imu_data[i].time < time_begin) {
            data_format::imu_type data = interpolate_imu(imu_data[i], imu_data[i + 1], time_begin);
            prop_data.push_back(data);
            continue;
        }

        // IMU is within time_begin and time_end
        if (imu_data[i].time >= time_begin && imu_data[i + 1].time <= time_end) {
            prop_data.push_back(imu_data[i]);
            continue;
        }

        // IMU is past time_end
        if (imu_data[i + 1].time > time_end) {
            data_format::imu_type data = interpolate_imu(imu_data[i], imu_data[i + 1], time_end);
            prop_data.push_back(data);
            break;
        }
    }

    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (int i = 0; i < int(prop_data.size()) - 1; i++) {
        // I need prop_data.size() - 1 to be signed, because it might equal -1.
        if (std::chrono::abs(prop_data[i + 1].time - prop_data[i].time) <
            std::chrono::nanoseconds{1}) {
            prop_data.erase(prop_data.begin() + i);
            i--;
        }
    }

    return prop_data;
}

// For when an integration time ever falls inbetween two imu measurements (modeled after OpenVINS)
data_format::imu_type gtsam_integrator::interpolate_imu(const data_format::imu_type& imu_1,
                                                        const data_format::imu_type& imu_2,
                                                        time_point timestamp) {
    double lambda =
            duration_to_double(timestamp - imu_1.time) / duration_to_double(imu_2.time - imu_1.time);
    return data_format::imu_type{timestamp, (1 - lambda) * imu_1.linear_a + lambda * imu_2.linear_a,
                                 (1 - lambda) * imu_1.angular_v + lambda * imu_2.angular_v};
}

PLUGIN_MAIN(gtsam_integrator)
