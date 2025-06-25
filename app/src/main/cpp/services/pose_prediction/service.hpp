#pragma once

#include "illixr/data_format/imu.hpp"
#include "illixr/data_format/pose_prediction.hpp"

namespace ILLIXR {
class pose_prediction_impl : public data_format::pose_prediction {
public:
    pose_prediction_impl(const phonebook* const pb);

    // No parameter get_fast_pose() should just predict to the next vsync
    // However, we don't have vsync estimation yet.
    // So we will predict to `now()`, as a temporary approximation
    data_format::fast_pose_type get_fast_pose() const override;

    data_format::pose_type get_true_pose() const override;

    // future_time: An absolute timepoint in the future
    data_format::fast_pose_type get_fast_pose(time_point future_timestamp) const override;

    void set_offset(const Eigen::Quaternionf& raw_o_times_offset) override;

    Eigen::Quaternionf apply_offset(const Eigen::Quaternionf& orientation) const;

    bool fast_pose_reliable() const override {
        /*
  SLAM takes some time to initialize, so initially fast_pose
  is unreliable.

  In such cases, we might return a fast_pose based only on the
  IMU data (currently, we just return a zero-pose)., and mark
  it as "unreliable"

  This way, there always a pose coming out of pose_prediction,
  representing our best guess at that time, and we indicate
  how reliable that guess is here.

 */

        return slow_pose_.get_ro_nullable() && imu_raw_.get_ro_nullable();
    }

    bool true_pose_reliable() const override {
        // return _m_true_pose.valid();
        /*
          We do not have a "ground truth" available in all cases, such
          as when reading live data.
         */
        return bool(true_pose_.get_ro_nullable());
    }

    Eigen::Quaternionf get_offset() override {
        return offset_;
    }

    // Correct the orientation of the pose due to the lopsided IMU in the
    // current Dataset we are using (EuRoC)
    data_format::pose_type correct_pose(const data_format::pose_type pose) const override;

private:
    mutable std::atomic<bool> first_time_{true};
    const std::shared_ptr<switchboard> switchboard_;
    const std::shared_ptr<const RelativeClock> clock_;
    switchboard::reader<data_format::pose_type> slow_pose_;
    switchboard::reader<data_format::imu_raw_type> imu_raw_;
    switchboard::reader<data_format::pose_type> true_pose_;
    switchboard::reader<switchboard::event_wrapper<Eigen::Vector3f>> ground_truth_offset_;
    switchboard::reader<switchboard::event_wrapper<time_point>> vsync_estimate_;
    mutable Eigen::Quaternionf offset_{Eigen::Quaternionf::Identity()};
    mutable std::shared_mutex offset_mutex_;

    // Slightly modified copy of OpenVINS method found in propagator.cpp
    // Returns a pair of the predictor state_plus and the time associated with the
    // most recent imu reading used to perform this prediction.
    std::pair<Eigen::Matrix<double, 13, 1>, time_point> predict_mean_rk4(double dt) const;

    /**
     * @brief Integrated quaternion from angular velocity
     *
     * See equation (48) of trawny tech report [Indirect Kalman Filter for 3D Attitude
     * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
     *
     */
    static const inline Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w);

    /**
     * @brief Normalizes a quaternion to make sure it is unit norm
     * @param q_t Quaternion to normalized
     * @return Normalized quaterion
     */
    static const inline Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t);

    /**
     * @brief Skew-symmetric matrix from a given 3x1 vector
     *
     * This is based on equation 6 in [Indirect Kalman Filter for 3D Attitude
     * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf): \f{align*}{ \lfloor\mathbf{v}\times\rfloor =
     *  \begin{bmatrix}
     *  0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0
     *  \end{bmatrix}
     * @f}
     *
     * @param[in] w 3x1 vector to be made a skew-symmetric
     * @return 3x3 skew-symmetric matrix
     */
    static const inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1>& w);

    /**
     * @brief Converts JPL quaterion to SO(3) rotation matrix
     *
     * This is based on equation 62 in [Indirect Kalman Filter for 3D Attitude
     * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf): \f{align*}{ \mathbf{R} =
     * (2q_4^2-1)\mathbf{I}_3-2q_4\lfloor\mathbf{q}\times\rfloor+2\mathbf{q}^\top\mathbf{q}
     * @f}
     *
     * @param[in] q JPL quaternion
     * @return 3x3 SO(3) rotation matrix
     */
    static const inline Eigen::Matrix<double, 3, 3>
    quat_2_Rot(const Eigen::Matrix<double, 4, 1>& q);

    /**
     * @brief Multiply two JPL quaternions
     *
     * This is based on equation 9 in [Indirect Kalman Filter for 3D Attitude
     * Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf). We also enforce that the quaternion is unique by having q_4
     * be greater than zero. \f{align*}{ \bar{q}\otimes\bar{p}= \mathcal{L}(\bar{q})\bar{p}= \begin{bmatrix}
     *  q_4\mathbf{I}_3+\lfloor\mathbf{q}\times\rfloor & \mathbf{q} \\
     *  -\mathbf{q}^\top & q_4
     *  \end{bmatrix}
     *  \begin{bmatrix}
     *  \mathbf{p} \\ p_4
     *  \end{bmatrix}
     * @f}
     *
     * @param[in] q First JPL quaternion
     * @param[in] p Second JPL quaternion
     * @return 4x1 resulting p*q quaternion
     */
    static const inline Eigen::Matrix<double, 4, 1>
    quat_multiply(const Eigen::Matrix<double, 4, 1>& q,
                  const Eigen::Matrix<double, 4, 1>& p);
};

}