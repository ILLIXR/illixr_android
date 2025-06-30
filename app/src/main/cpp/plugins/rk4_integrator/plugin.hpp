#pragma once

#include "illixr/plugin.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format/imu.hpp"

namespace ILLIXR {

class rk4_integrator : public plugin {
public:
    [[maybe_unused]] rk4_integrator(const std::string& name_, phonebook* pb_);

    void callback(switchboard::ptr<const data_format::imu_type> datum);

private:
    const std::shared_ptr<switchboard> switchboard_;
//    const std::shared_ptr<log_service> sl;

    // IMU Data, Sequence Flag, and State Vars Needed
    switchboard::reader<data_format::imu_integrator_input> imu_integrator_input_;

    // IMU Biases
    switchboard::writer<data_format::imu_raw_type> imu_raw_;
    std::vector<data_format::imu_type>             imu_vec_;
    duration                          last_imu_offset_;
    bool                              has_last_offset_ = false;

    [[maybe_unused]] int    counter_       = 0;
    [[maybe_unused]] int    cam_count_     = 0;
    [[maybe_unused]] int    total_imu_     = 0;
    [[maybe_unused]] double last_cam_time_ = 0;

    // Clean IMU values older than IMU_SAMPLE_LIFETIME seconds
    void clean_imu_vec(time_point timestamp);

    // Timestamp we are propagating the biases to (new IMU reading time)
    void propagate_imu_values(time_point real_time);

    // Select IMU readings based on timestamp similar to how OpenVINS selects IMU values to propagate
    std::vector<data_format::imu_type> select_imu_readings(const std::vector<data_format::imu_type>& imu_data, time_point time_begin,
                                                           time_point time_end);

    // For when an integration time ever falls inbetween two imu measurements (modeled after OpenVINS)
    static data_format::imu_type interpolate_imu(const data_format::imu_type& imu_1, const data_format::imu_type& imu_2, time_point timestamp);

    void predict_mean_rk4(Eigen::Vector4d quat, Eigen::Vector3d pos, Eigen::Vector3d vel, double dt,
                          const Eigen::Vector3d& w_hat1, const Eigen::Vector3d& a_hat1, const Eigen::Vector3d& w_hat2,
                          const Eigen::Vector3d& a_hat2, Eigen::Vector4d& new_q, Eigen::Vector3d& new_v,
                          Eigen::Vector3d& new_p);

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
    static const inline Eigen::Matrix<double, 3, 3> quat_2_Rot(const Eigen::Matrix<double, 4, 1>& q);

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
    static const inline Eigen::Matrix<double, 4, 1> quat_multiply(const Eigen::Matrix<double, 4, 1>& q,
                                                                  const Eigen::Matrix<double, 4, 1>& p);
};

}