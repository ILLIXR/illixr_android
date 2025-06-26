#include "service.hpp"

#include "illixr/plugin.hpp"

#include <Eigen/Dense>
#include <shared_mutex>
#include <android/log.h>
#include <fstream>

std::ofstream myfile;
#define ANDROID_LOG(...) ((void)__android_log_print(ANDROID_LOG_INFO, "pose_prediction", __VA_ARGS__))

using namespace ILLIXR;

pose_prediction_impl::pose_prediction_impl(const phonebook* const pb)
        : switchboard_{pb->lookup_impl<switchboard>()}, clock_{pb->lookup_impl<RelativeClock>()},
          slow_pose_{switchboard_->get_reader<data_format::pose_type>("slow_pose")},
          imu_raw_{switchboard_->get_reader<ILLIXR::data_format::imu_raw_type>("imu_raw")},
          true_pose_{switchboard_->get_reader<data_format::pose_type>("true_pose")}, ground_truth_offset_{
                switchboard_->get_reader<switchboard::event_wrapper<Eigen::Vector3f>>("ground_truth_offset")},
          vsync_estimate_{
                  switchboard_->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")} {
    myfile.open("/sdcard/Android/data/com.example.native_activity/pose_prediction.tum");
}

data_format::fast_pose_type pose_prediction_impl::get_fast_pose() const {
    switchboard::ptr<const switchboard::event_wrapper<time_point>> vsync_estimate = vsync_estimate_.get_ro_nullable();
    // ANDROID_LOG("GET FAST POSE");
    if (vsync_estimate == nullptr) {
        //ANDROID_LOG("with clock");
        return get_fast_pose(clock_->now());
    } else {
        //ANDROID_LOG("GET FAST POSE");
        return get_fast_pose(*vsync_estimate);
    }
}

data_format::pose_type pose_prediction_impl::get_true_pose() const {
    switchboard::ptr<const data_format::pose_type> pose_ptr = true_pose_.get_ro_nullable();
    switchboard::ptr<const switchboard::event_wrapper<Eigen::Vector3f>> offset_ptr =
            ground_truth_offset_.get_ro_nullable();

    data_format::pose_type offset_pose;

    // Subtract offset if valid pose and offset, otherwise use zero pose.
    // Checking that pose and offset are both valid is safer than just
    // checking one or the other because it assumes nothing about the
    // ordering of writes on the producer's end or about the producer
    // actually writing to both streams.
    if (pose_ptr != nullptr && offset_ptr != nullptr) {
        offset_pose = *pose_ptr;
        offset_pose.position -= **offset_ptr;
    } else {
        offset_pose.sensor_time = clock_->now();
        offset_pose.position = Eigen::Vector3f{0, 0, 0};
        offset_pose.orientation = Eigen::Quaternionf{1, 0, 0, 0};
    }
    return correct_pose(offset_pose);
}

// future_time: An absolute timepoint in the future
data_format::fast_pose_type pose_prediction_impl::get_fast_pose(time_point future_timestamp) const {
    switchboard::ptr<const data_format::pose_type> slow_pose = slow_pose_.get_ro_nullable();


    if (slow_pose == nullptr) {
        // No slow pose, return 0
        return data_format::fast_pose_type{
                correct_pose(data_format::pose_type{}),
                clock_->now(),
                future_timestamp,
        };
    }

    switchboard::ptr<const data_format::imu_raw_type> imu_raw = imu_raw_.get_ro_nullable();
    if (imu_raw == nullptr) {
#ifndef NDEBUG
        ANDROID_LOG("FAST POSE IS SLOW POSE!\n");
#endif
        // No imu_raw, return slow_pose
        return data_format::fast_pose_type{
                .pose                  = correct_pose(*slow_pose),
                .predict_computed_time = clock_->now(),
                .predict_target_time   = future_timestamp,
        };
    }

    //Return SLOW POSE
//        if (first_time) {
//            pose_type first_pose = correct_pose(*slow_pose);
//            std::unique_lock lock{offset_mutex};
//            // check again, now that we have mutual exclusion
//            if (first_time) {
//                first_time = false;
//                offset     = first_pose.orientation.inverse();
//            }
//        }
//
//        return fast_pose_type{
//                correct_pose(*slow_pose),
//                clock_->now(),
//                future_timestamp,
//        };
//        // slow_pose and imu_raw, do pose prediction

    double dt = duration2double(future_timestamp - imu_raw->imu_time);
    std::pair<Eigen::Matrix<double, 13, 1>, time_point> predictor_result = predict_mean_rk4(dt);

    auto state_plus = predictor_result.first;

    // predictor_imu_time is the most recent IMU sample that was used to compute the prediction.
    auto predictor_imu_time = predictor_result.second;

    data_format::pose_type predicted_pose = correct_pose({
                                                                 predictor_imu_time,
                                                                 Eigen::Vector3f{
                                                                         static_cast<float>(state_plus(
                                                                                 4)),
                                                                         static_cast<float>(state_plus(
                                                                                 5)),
                                                                         static_cast<float>(state_plus(
                                                                                 6))
                                                                 },
                                                                 Eigen::Quaternionf{
                                                                         static_cast<float>(state_plus(
                                                                                 3)),
                                                                         static_cast<float>(state_plus(
                                                                                 0)),
                                                                         static_cast<float>(state_plus(
                                                                                 1)),
                                                                         static_cast<float>(state_plus(
                                                                                 2))
                                                                 }
                                                         });

    // Make the first valid fast pose be straight ahead.
    if (first_time_) {
        std::unique_lock lock{offset_mutex_};
        // check again, now that we have mutual exclusion
        if (first_time_) {
            first_time_ = false;
            offset_ = predicted_pose.orientation.inverse();
        }
    }

//        ANDROID_LOG("POse position = %f %f %f , prediction orientation = %f %f %f %f",
//             predicted_pose.position.x(), predicted_pose.position.y(), predicted_pose.position.z()
//             ,predicted_pose.orientation.w(), predicted_pose.orientation.x(), predicted_pose.orientation.y(), predicted_pose.orientation.z());
    // Several timestamps are logged:
    //       - the prediction compute time (time when this prediction was computed, i.e., now)
    //       - the prediction target (the time that was requested for this pose.)

    return data_format::fast_pose_type{
            .pose = predicted_pose,
            .predict_computed_time = clock_->now(),
            .predict_target_time = future_timestamp
    };
}

void pose_prediction_impl::set_offset(const Eigen::Quaternionf& raw_o_times_offset) {
    std::unique_lock lock{offset_mutex_};
    Eigen::Quaternionf raw_o = raw_o_times_offset * offset_.inverse();
    offset_ = raw_o.inverse();
    /*
      Now, `raw_o` is maps to the identity quaternion.
      Proof:
      apply_offset(raw_o)
          = raw_o * offset
          = raw_o * raw_o.inverse()
          = Identity.
     */
}

Eigen::Quaternionf pose_prediction_impl::apply_offset(const Eigen::Quaternionf& orientation) const {
    std::shared_lock lock{offset_mutex_};
    return orientation * offset_;
}


// Correct the orientation of the pose due to the lopsided IMU in the
// current Dataset we are using (EuRoC)
data_format::pose_type pose_prediction_impl::correct_pose(const data_format::pose_type pose) const {
    data_format::pose_type swapped_pose;
    //ANDROID_LOG("CORRECT POSE");
    // Make any changes to the axes direction below
    // This is a mapping between the coordinate system of the current
    // SLAM (OpenVINS) we are using and the OpenGL system.
    swapped_pose.position.x() = -pose.position.y();//+ 2.28 ;
    swapped_pose.position.y() = pose.position.z();//- 1;
    swapped_pose.position.z() = -pose.position.x();//+ 0.49;

    // Make any chanes to orientation of the output below
    // For the dataset were currently using (EuRoC), the output orientation acts as though
    // the "top of the head" is the forward direction, and the "eye direction" is the up direction.
    Eigen::Quaternionf raw_o(pose.orientation.w(), -pose.orientation.y(), pose.orientation.z(),
                             -pose.orientation.x());

    swapped_pose.orientation = apply_offset(raw_o);
    swapped_pose.sensor_time = pose.sensor_time;
    myfile << std::to_string(duration2double(swapped_pose.sensor_time.time_since_epoch())) + " " +
              std::to_string(swapped_pose.position.x()) + " " +
              std::to_string(swapped_pose.position.y()) + " " +
              std::to_string(swapped_pose.position.z())
              + " " + std::to_string(swapped_pose.orientation.x()) + " " +
              std::to_string(swapped_pose.orientation.y()) + " " +
              std::to_string(swapped_pose.orientation.z()) + " " +
              std::to_string(swapped_pose.orientation.w()) + "\n";
    return swapped_pose;
}

// Slightly modified copy of OpenVINS method found in propagator.cpp
// Returns a pair of the predictor state_plus and the time associated with the
// most recent imu reading used to perform this prediction.
std::pair<Eigen::Matrix<double, 13, 1>, time_point>
pose_prediction_impl::predict_mean_rk4(double dt) const {
    // Pre-compute things
    switchboard::ptr<const data_format::imu_raw_type> imu_raw = imu_raw_.get_ro();
    //ANDROID_LOG("Read imu data ..");

    Eigen::Vector3d w_hat = imu_raw->w_hat;
    Eigen::Vector3d a_hat = imu_raw->a_hat;
    Eigen::Vector3d w_alpha = (imu_raw->w_hat2 - imu_raw->w_hat) / dt;
    Eigen::Vector3d a_jerk = (imu_raw->a_hat2 - imu_raw->a_hat) / dt;

    // y0 ================
    Eigen::Quaterniond temp_quat = imu_raw->quat;
    Eigen::Vector4d q_0 = {temp_quat.x(), temp_quat.y(), temp_quat.z(), temp_quat.w()};
    Eigen::Vector3d p_0 = imu_raw->pos;
    Eigen::Vector3d v_0 = imu_raw->vel;

    // k1 ================
    Eigen::Vector4d dq_0 = {0, 0, 0, 1};
    Eigen::Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
    Eigen::Vector3d p0_dot = v_0;
    Eigen::Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
    Eigen::Vector3d v0_dot = R_Gto0.transpose() * a_hat - Eigen::Vector3d{0.0, 0.0, 9.81};

    Eigen::Vector4d k1_q = q0_dot * dt;
    Eigen::Vector3d k1_p = p0_dot * dt;
    Eigen::Vector3d k1_v = v0_dot * dt;

    // k2 ================
    w_hat += 0.5 * w_alpha * dt;
    a_hat += 0.5 * a_jerk * dt;

    Eigen::Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
    Eigen::Vector3d v_1 = v_0 + 0.5 * k1_v;

    Eigen::Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
    Eigen::Vector3d p1_dot = v_1;
    Eigen::Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
    Eigen::Vector3d v1_dot = R_Gto1.transpose() * a_hat - Eigen::Vector3d{0.0, 0.0, 9.81};

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
    Eigen::Vector3d v2_dot = R_Gto2.transpose() * a_hat - Eigen::Vector3d{0.0, 0.0, 9.81};

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
    Eigen::Vector3d v3_dot = R_Gto3.transpose() * a_hat - Eigen::Vector3d{0.0, 0.0, 9.81};

    Eigen::Vector4d k4_q = q3_dot * dt;
    Eigen::Vector3d k4_p = p3_dot * dt;
    Eigen::Vector3d k4_v = v3_dot * dt;

    // y+dt ================
    Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();
    Eigen::Vector4d dq = quatnorm(
            dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q +
            (1.0 / 6.0) * k4_q);
    state_plus.block(0, 0, 4, 1) = quat_multiply(dq, q_0);
    state_plus.block(4, 0, 3, 1) =
            p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
    state_plus.block(7, 0, 3, 1) =
            v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;

    return {state_plus, imu_raw->imu_time};
}

const Eigen::Matrix<double, 4, 4> pose_prediction_impl::Omega(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 4, 4> mat;
    mat.block(0, 0, 3, 3) = -skew_x(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
}

const Eigen::Matrix<double, 4, 1> pose_prediction_impl::quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
    if (q_t(3, 0) < 0) {
        q_t *= -1;
    }
    return q_t / q_t.norm();
}

const Eigen::Matrix<double, 3, 3>
pose_prediction_impl::skew_x(const Eigen::Matrix<double, 3, 1>& w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    return w_x;
}

const Eigen::Matrix<double, 3, 3>
pose_prediction_impl::quat_2_Rot(const Eigen::Matrix<double, 4, 1>& q) {
    Eigen::Matrix<double, 3, 3> q_x = skew_x(q.block(0, 0, 3, 1));
    Eigen::MatrixXd Rot = (2 * std::pow(q(3, 0), 2) - 1) * Eigen::MatrixXd::Identity(3, 3)
                          - 2 * q(3, 0) * q_x +
                          2 * q.block(0, 0, 3, 1) * (q.block(0, 0, 3, 1).transpose());
    return Rot;
}

const Eigen::Matrix<double, 4, 1>
pose_prediction_impl::quat_multiply(const Eigen::Matrix<double, 4, 1>& q,
                                    const Eigen::Matrix<double, 4, 1>& p) {
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


class pose_prediction_plugin : public plugin {
public:
    [[maybe_unused]] pose_prediction_plugin(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        pb->register_impl<data_format::pose_prediction>(
                std::static_pointer_cast<data_format::pose_prediction>(
                        std::make_shared<pose_prediction_impl>(pb)));
    }
};

PLUGIN_MAIN(pose_prediction_plugin)
