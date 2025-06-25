// This entire IMU integrator has been ported almost as-is from the original OpenVINS integrator, which
// can be found here: https://github.com/rpng/open_vins/blob/master/ov_msckf/src/state/Propagator.cpp

#include "plugin.hpp"

#include <chrono>
#include <thread>
#include <android/log.h>

using namespace ILLIXR;

constexpr duration IMU_SAMPLE_LIFETIME{std::chrono::seconds{2}};

#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "rk4", __VA_ARGS__))


[[maybe_unused]] rk4_integrator::rk4_integrator(const std::string &name_, phonebook *pb_)
        : plugin{name_, pb_}, switchboard_{pb->lookup_impl<switchboard>()}
//            , sl{pb->lookup_impl<log_service>()}
        , imu_integrator_input_{
                switchboard_->get_reader<data_format::imu_integrator_input>(
                        "imu_integrator_input")},
          imu_raw_{switchboard_->get_writer<data_format::imu_raw_type>("imu_raw")} {
    ANDROID_LOG("RK4 INTEGRATOR");
    switchboard_->schedule<data_format::imu_cam_type>(id, "imu_cam",
                                                      [&](switchboard::ptr<const data_format::imu_cam_type> datum,
                                                          size_t) {
                                                          callback(datum);
                                                      });
}

void rk4_integrator::callback(switchboard::ptr<const data_format::imu_cam_type> datum) {
    imu_vec_.emplace_back(datum->time, datum->angular_v.cast<double>(),
                          datum->linear_a.cast<double>());

    clean_imu_vec(datum->time);
    auto start = std::chrono::high_resolution_clock::now();
    propagate_imu_values(datum->time);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    ANDROID_LOG("duration: %f", duration2double(duration));
//        sl->write_duration("rk4", duration2double(duration));
    RAC_ERRNO_MSG("rk4_integrator");
}

// Clean IMU values older than IMU_SAMPLE_LIFETIME seconds
void rk4_integrator::clean_imu_vec(time_point timestamp) {
    auto it0 = imu_vec_.begin();
    while (it0 != imu_vec_.end()) {
        if (timestamp - it0->timestamp < IMU_SAMPLE_LIFETIME) {
            break;
        }
        it0 = imu_vec_.erase(it0);
    }
}

// Timestamp we are propagating the biases to (new IMU reading time)
void rk4_integrator::propagate_imu_values(time_point real_time) {
    auto input_values = imu_integrator_input_.get_ro_nullable();
    if (input_values == nullptr) {
        return;
    }

    if (!has_last_offset_) {
        /// TODO: Should be set and tested at the end of this function to avoid staleness from VIO.
        last_imu_offset_ = input_values->t_offset;
        has_last_offset_ = true;
    }

    Eigen::Matrix<double, 4, 1> curr_quat = Eigen::Matrix<double, 4, 1>{input_values->quat.x(),
                                                                        input_values->quat.y(),
                                                                        input_values->quat.z(),
                                                                        input_values->quat.w()};
    Eigen::Matrix<double, 3, 1> curr_pos = input_values->position;
    Eigen::Matrix<double, 3, 1> curr_vel = input_values->velocity;

    // Uncomment this for some helpful prints
    // total_imu++;
    // if (input_values->last_cam_integration_time > last_cam_time) {
    // 	cam_count++;
    // 	last_cam_time = input_values->last_cam_integration_time;
    // 	std::cout << "Num IMUs recieved since last cam: " << counter << " Diff between new cam and latest IMU: "
    // 			  << timestamp - last_cam_time << " Expected IMUs recieved VS Actual: " << cam_count*10 << ", " << total_imu
    // << std::endl; 	counter = 0;
    // }
    // counter++;

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    duration t_off_new = input_values->t_offset;

    // This is the last CAM time
    time_point time0 = input_values->last_cam_integration_time + last_imu_offset_;
    time_point time1 = real_time + t_off_new;

    std::vector<data_format::imu_type> prop_data = select_imu_readings(imu_vec_, time0, time1);
    Eigen::Matrix<double, 3, 1> w_hat;
    Eigen::Matrix<double, 3, 1> a_hat;
    Eigen::Matrix<double, 3, 1> w_hat2;
    Eigen::Matrix<double, 3, 1> a_hat2;

    // Loop through all IMU messages, and use them to move the state forward in time
    // This uses the zero'th order quat, and then constant acceleration discrete
    if (prop_data.size() > 1) {
        for (size_t i = 0; i < prop_data.size() - 1; i++) {
            // Time elapsed over interval
            double dt = duration2double(prop_data[i + 1].timestamp - prop_data[i].timestamp);

            // Corrected imu measurements
            w_hat = prop_data[i].wm - input_values->biasGyro;
            a_hat = prop_data[i].am - input_values->biasAcc;
            w_hat2 = prop_data[i + 1].wm - input_values->biasGyro;
            a_hat2 = prop_data[i + 1].am - input_values->biasAcc;

            // Compute the new state mean value
            Eigen::Vector4d new_quat;
            Eigen::Vector3d new_vel, new_pos;
            predict_mean_rk4(curr_quat, curr_pos, curr_vel, dt, w_hat, a_hat, w_hat2, a_hat2,
                             new_quat, new_vel, new_pos);

            curr_quat = new_quat;
            curr_pos = new_pos;
            curr_vel = new_vel;
        }
    }

    imu_raw_.put(imu_raw_.allocate(w_hat, a_hat, w_hat2, a_hat2, curr_pos, curr_vel,
                                   Eigen::Quaterniond{curr_quat(3), curr_quat(0), curr_quat(1),
                                                      curr_quat(2)},
                                   real_time));
}

// Select IMU readings based on timestamp similar to how OpenVINS selects IMU values to propagate
std::vector<data_format::imu_type>
rk4_integrator::select_imu_readings(const std::vector<data_format::imu_type> &imu_data,
                                    time_point time_begin,
                                    time_point time_end) {
    std::vector<data_format::imu_type> prop_data;
    if (imu_data.size() < 2) {
        return prop_data;
    }

    for (size_t i = 0; i < imu_data.size() - 1; i++) {
        // If time_begin comes inbetween two IMUs (A and B), interpolate A forward to time_begin
        if (imu_data[i + 1].timestamp > time_begin && imu_data[i].timestamp < time_begin) {
            data_format::imu_type data = interpolate_imu(imu_data[i], imu_data[i + 1], time_begin);
            prop_data.push_back(data);
            continue;
        }

        // IMU is within time_begin and time_end
        if (imu_data[i].timestamp >= time_begin && imu_data[i + 1].timestamp <= time_end) {
            prop_data.push_back(imu_data[i]);
            continue;
        }

        // IMU is past time_end
        if (imu_data[i + 1].timestamp > time_end) {
            data_format::imu_type data = interpolate_imu(imu_data[i], imu_data[i + 1], time_end);
            prop_data.push_back(data);
            break;
        }
    }

    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (int i = 0; i < int(prop_data.size()) - 1; i++) {
        if (std::chrono::abs(prop_data[i + 1].timestamp - prop_data[i].timestamp) <
            std::chrono::nanoseconds{1}) {
            prop_data.erase(prop_data.begin() + i);
            i--; // i can be negative, so use type int
        }
    }

    return prop_data;
}

// For when an integration time ever falls inbetween two imu measurements (modeled after OpenVINS)
data_format::imu_type rk4_integrator::interpolate_imu(const data_format::imu_type &imu_1,
                                                      const data_format::imu_type &imu_2,
                                                      time_point timestamp) {
    double lambda = duration2double(timestamp - imu_1.timestamp) /
                    duration2double(imu_2.timestamp - imu_1.timestamp);
    return data_format::imu_type{timestamp, (1 - lambda) * imu_1.am + lambda * imu_2.am,
                                 (1 - lambda) * imu_1.wm + lambda * imu_2.wm};
}

void
rk4_integrator::predict_mean_rk4(Eigen::Vector4d quat, Eigen::Vector3d pos, Eigen::Vector3d vel,
                                 double dt,
                                 const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                 const Eigen::Vector3d &w_hat2,
                                 const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q,
                                 Eigen::Vector3d &new_v,
                                 Eigen::Vector3d &new_p) {
    Eigen::Matrix<double, 3, 1> gravity_vec = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 9.81);

    // Pre-compute things
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    Eigen::Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
    Eigen::Vector3d a_jerk = (a_hat2 - a_hat1) / dt;

    // y0 ================
    Eigen::Vector4d q_0 = quat;
    Eigen::Vector3d p_0 = pos;
    Eigen::Vector3d v_0 = vel;

    // k1 ================
    Eigen::Vector4d dq_0 = {0, 0, 0, 1};
    Eigen::Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
    Eigen::Vector3d p0_dot = v_0;
    Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
    Eigen::Vector3d v0_dot = R_Gto0.transpose() * a_hat - gravity_vec;

    Eigen::Vector4d k1_q = q0_dot * dt;
    Eigen::Vector3d k1_p = p0_dot * dt;
    Eigen::Vector3d k1_v = v0_dot * dt;

    // k2 ================
    w_hat += 0.5 * w_alpha * dt;
    a_hat += 0.5 * a_jerk * dt;

    Eigen::Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
    // Eigen::Vector3d p_1 = p_0+0.5*k1_p;
    Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;

    Eigen::Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
    Eigen::Vector3d p1_dot = v_1;
    Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
    Eigen::Vector3d v1_dot = R_Gto1.transpose() * a_hat - gravity_vec;

    Eigen::Vector4d k2_q = q1_dot * dt;
    Eigen::Vector3d k2_p = p1_dot * dt;
    Eigen::Vector3d k2_v = v1_dot * dt;

    // k3 ================
    Eigen::Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
    // Eigen::Vector3d p_2 = p_0+0.5*k2_p;
    Eigen::Vector3d v_2 = v_0 + 0.5 * k2_v;

    Eigen::Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
    Eigen::Vector3d p2_dot = v_2;
    Eigen::Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_0));
    Eigen::Vector3d v2_dot = R_Gto2.transpose() * a_hat - gravity_vec;

    Eigen::Vector4d k3_q = q2_dot * dt;
    Eigen::Vector3d k3_p = p2_dot * dt;
    Eigen::Vector3d k3_v = v2_dot * dt;

    // k4 ================
    w_hat += 0.5 * w_alpha * dt;
    a_hat += 0.5 * a_jerk * dt;

    Eigen::Vector4d dq_3 = quatnorm(dq_0 + k3_q);
    // Eigen::Vector3d p_3 = p_0+k3_p;
    Eigen::Vector3d v_3 = v_0 + k3_v;

    Eigen::Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
    Eigen::Vector3d p3_dot = v_3;
    Eigen::Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_0));
    Eigen::Vector3d v3_dot = R_Gto3.transpose() * a_hat - gravity_vec;

    Eigen::Vector4d k4_q = q3_dot * dt;
    Eigen::Vector3d k4_p = p3_dot * dt;
    Eigen::Vector3d k4_v = v3_dot * dt;

    // y+dt ================
    Eigen::Vector4d dq = quatnorm(
            dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q +
            (1.0 / 6.0) * k4_q);
    new_q = quat_multiply(dq, q_0);
    new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
    new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
}

const Eigen::Matrix<double, 4, 4> rk4_integrator::Omega(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 4, 4> mat;
    mat.block(0, 0, 3, 3) = -skew_x(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
}

const Eigen::Matrix<double, 4, 1> rk4_integrator::quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
    if (q_t(3, 0) < 0) {
        q_t *= -1;
    }
    return q_t / q_t.norm();
}

const Eigen::Matrix<double, 3, 3> rk4_integrator::skew_x(const Eigen::Matrix<double, 3, 1> &w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
}

const Eigen::Matrix<double, 3, 3> rk4_integrator::quat_2_Rot(const Eigen::Matrix<double, 4, 1> &q) {
    Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
    Eigen::MatrixXd Rot =
            (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3) - 2 * q(3, 0) * q_x +
            2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
    return Rot;
}

const Eigen::Matrix<double, 4, 1>
rk4_integrator::quat_multiply(const Eigen::Matrix<double, 4, 1> &q,
                              const Eigen::Matrix<double, 4, 1> &p) {
    Eigen::Matrix<double, 4, 1> q_t;
    Eigen::Matrix<double, 4, 4> Qm;
    // create big L matrix
    Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
    Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
    Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
    Qm(3, 3) = q(3, 0);
    q_t = Qm * p;
    // ensure unique by forcing q_4 to be >0
    if (q_t(3, 0) < 0) {
        q_t *= -1;
    }
    // normalize and return
    return q_t / q_t.norm();
}

PLUGIN_MAIN(rk4_integrator)