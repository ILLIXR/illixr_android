#pragma once

#include "illixr/switchboard.hpp"
#include <opencv2/core/mat.hpp>
#include <Eigen/Dense>

namespace ILLIXR::data_format {
    // Data type that combines the IMU and camera data at a certain timestamp.
// If there is only IMU data for a certain timestamp, img0 and img1 will be null
// time is the current UNIX time where dataset_time is the time read from the csv
    struct imu_cam_type : public switchboard::event {
        time_point             time;
        Eigen::Vector3f        angular_v;
        Eigen::Vector3f        linear_a;
        std::optional<cv::Mat> img0;
        std::optional<cv::Mat> img1;
        imu_cam_type(time_point time_,
                     Eigen::Vector3f angular_v_,
                     Eigen::Vector3f linear_a_,
                     std::optional<cv::Mat> img0_,
                     std::optional<cv::Mat> img1_)
                : time{time_}
                , angular_v{angular_v_}
                , linear_a{linear_a_}
                , img0{img0_}
                , img1{img1_}
        { }
    };

    struct imu_type {
        time_point                  timestamp;
        Eigen::Matrix<double, 3, 1> wm;
        Eigen::Matrix<double, 3, 1> am;


        imu_type(
                time_point timestamp_,
                Eigen::Matrix<double, 3, 1> wm_,
                Eigen::Matrix<double, 3, 1> am_
        )
                : timestamp{timestamp_}
                , wm{wm_}
                , am{am_}
        { }
    };

    // Values needed to initialize the IMU integrator
    typedef struct {
        double                      gyro_noise;
        double                      acc_noise;
        double                      gyro_walk;
        double                      acc_walk;
        Eigen::Matrix<double, 3, 1> n_gravity;
        double                      imu_integration_sigma;
        double                      nominal_rate;
    } imu_params;

// IMU biases, initialization params, and slow pose needed by the IMU integrator
    struct imu_integrator_input : public switchboard::event {
        time_point last_cam_integration_time;
        duration   t_offset;
        imu_params params;

        Eigen::Vector3d             biasAcc;
        Eigen::Vector3d             biasGyro;
        Eigen::Matrix<double, 3, 1> position;
        Eigen::Matrix<double, 3, 1> velocity;
        Eigen::Quaterniond          quat;
        imu_integrator_input(
                time_point last_cam_integration_time_,
                duration t_offset_,
                imu_params params_,
                Eigen::Vector3d biasAcc_,
                Eigen::Vector3d biasGyro_,
                Eigen::Matrix<double,3,1> position_,
                Eigen::Matrix<double,3,1> velocity_,
                Eigen::Quaterniond quat_
        )
                : last_cam_integration_time{last_cam_integration_time_}
                , t_offset{t_offset_}
                , params{params_}
                , biasAcc{biasAcc_}
                , biasGyro{biasGyro_}
                , position{position_}
                , velocity{velocity_}
                , quat{quat_}
        { }
    };

// Output of the IMU integrator to be used by pose prediction
    struct imu_raw_type : public switchboard::event {
        // Biases from the last two IMU integration iterations used by RK4 for pose predict
        Eigen::Matrix<double, 3, 1> w_hat;
        Eigen::Matrix<double, 3, 1> a_hat;
        Eigen::Matrix<double, 3, 1> w_hat2;
        Eigen::Matrix<double, 3, 1> a_hat2;

        // Faster pose propagated forwards by the IMU integrator
        Eigen::Matrix<double, 3, 1> pos;
        Eigen::Matrix<double, 3, 1> vel;
        Eigen::Quaterniond          quat;
        time_point                  imu_time;
        imu_raw_type(Eigen::Matrix<double,3,1> w_hat_,
                     Eigen::Matrix<double,3,1> a_hat_,
                     Eigen::Matrix<double,3,1> w_hat2_,
                     Eigen::Matrix<double,3,1> a_hat2_,
                     Eigen::Matrix<double,3,1> pos_,
                     Eigen::Matrix<double,3,1> vel_,
                     Eigen::Quaterniond quat_,
                     time_point imu_time_)
                : w_hat{w_hat_}
                , a_hat{a_hat_}
                , w_hat2{w_hat2_}
                , a_hat2{a_hat2_}
                , pos{pos_}
                , vel{vel_}
                , quat{quat_}
                , imu_time{imu_time_}
        { }
    };


}
