/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "common/plugin.hpp"

#include "common/data_format.hpp"
#include "common/phonebook.hpp"
#include "common/relative_clock.hpp"
#include "common/switchboard.hpp"
#include "core/VioManager.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "../ov_core/src/utils/quat_ops.h"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#define ANDROID_CAM 1
using namespace ov_msckf;
using namespace ILLIXR;

duration from_seconds(double seconds) {
    return duration{long(seconds * 1e9L)};
}

VioManagerOptions create_params()
{
    VioManagerOptions params;

    // Camera #0
    Eigen::Matrix<double, 8, 1> intrinsics_0;
#ifdef ZED
    // ZED calibration tool; fx, fy, cx, cy, k1, k2, p1, p2
  // https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
  intrinsics_0 << 349.686, 349.686, 332.778, 192.423, -0.175708, 0.0284421, 0, 0;
#elif ANDROID_CAM
    intrinsics_0 <<483.2087255, 484.10304, 372.136219, 242.353948, 0.040666, -0.053787, -2.796e-05,0.0079762;
#else
    // EuRoC
    intrinsics_0 << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
#endif

#ifdef ZED
    // Camera extrinsics from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  std::vector<double> matrix_TCtoI_0 = {-0.01080233, 0.00183858, 0.99993996, 0.01220425,
            -0.99993288, -0.00420947, -0.01079452, 0.0146056,
            0.00418937, -0.99998945, 0.00188393, -0.00113692,
            0.0, 0.0, 0.0, 1.0};
#elif ANDROID_CAM
    std::vector<double> matrix_TCtoI_0 = {0, -1, 0, 0,
                                          -1, 0, 0, 0,
                                          0, 0, -1, 0,
                                          0.0, 0.0, 0.0, 1.0};
#else
    std::vector<double> matrix_TCtoI_0 = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                                          0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                                          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                                          0.0, 0.0, 0.0, 1.0};
#endif

    Eigen::Matrix4d T_CtoI_0;
    T_CtoI_0 << matrix_TCtoI_0.at(0), matrix_TCtoI_0.at(1), matrix_TCtoI_0.at(2), matrix_TCtoI_0.at(3),
            matrix_TCtoI_0.at(4), matrix_TCtoI_0.at(5), matrix_TCtoI_0.at(6), matrix_TCtoI_0.at(7),
            matrix_TCtoI_0.at(8), matrix_TCtoI_0.at(9), matrix_TCtoI_0.at(10), matrix_TCtoI_0.at(11),
            matrix_TCtoI_0.at(12), matrix_TCtoI_0.at(13), matrix_TCtoI_0.at(14), matrix_TCtoI_0.at(15);

    // Load these into our state
    Eigen::Matrix<double, 7, 1> extrinsics_0;
    extrinsics_0.block(0, 0, 4, 1) = ov_core::rot_2_quat(T_CtoI_0.block(0, 0, 3, 3).transpose());
    extrinsics_0.block(4, 0, 3, 1) = -T_CtoI_0.block(0, 0, 3, 3).transpose() * T_CtoI_0.block(0, 3, 3, 1);

    //params.camera_fisheye.insert({0, false});
//    params.camera_intrinsics.insert({0, intrinsics_0});

#ifdef ZED
    params.camera_wh.insert({0, {672, 376}});
#else
    params.camera_intrinsics.insert({0, std::make_shared<ov_core::CamRadtan>(752, 480)});

//    params.camera_wh.insert({0, {752, 480}});
#endif
    params.camera_intrinsics.at(0)->set_value(intrinsics_0);
    params.camera_extrinsics.insert({0, extrinsics_0});

    // Camera #1
    Eigen::Matrix<double, 8, 1> intrinsics_1;
#ifdef ZED
    // ZED calibration tool; fx, fy, cx, cy, k1, k2, p1, p2
  intrinsics_1 << 350.01, 350.01, 343.729, 185.405, -0.174559, 0.0277521, 0, 0;
#elif ANDROID_CAM
    intrinsics_1 <<959.7108500223383, 958.2043029463723, 534.9459777106056, 724.9789679167809,  0.5891985645773625, -1.4239660996953498, 4.335816777863736, -4.151712962020906;
#else
    // EuRoC
    intrinsics_1 << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
#endif

#ifdef ZED
    // Camera extrinsics from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  std::vector<double> matrix_TCtoI_1 = {-0.01043535, -0.00191061, 0.99994372, 0.01190459,
            -0.99993668, -0.00419281, -0.01044329, -0.04732387,
            0.00421252, -0.99998938, -0.00186674, -0.00098799,
            0.0, 0.0, 0.0, 1.0};
#elif ANDROID_CAM
    std::vector<double> matrix_TCtoI_1 = {0.99999067, 0.00281849, 0.00327385, 0.01981983,
                                          0.00284178, -0.99997052, -0.00713377, -0.00464974,
                                          0.00325365, 0.00714301, -0.9999692, -0.01094682,
                                          0.0, 0.0, 0.0, 1.0};
#else
    std::vector<double> matrix_TCtoI_1 = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
                                          0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
                                          -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
                                          0.0, 0.0, 0.0, 1.0};
#endif

    Eigen::Matrix4d T_CtoI_1;
    T_CtoI_1 << matrix_TCtoI_1.at(0), matrix_TCtoI_1.at(1), matrix_TCtoI_1.at(2), matrix_TCtoI_1.at(3),
            matrix_TCtoI_1.at(4), matrix_TCtoI_1.at(5), matrix_TCtoI_1.at(6), matrix_TCtoI_1.at(7),
            matrix_TCtoI_1.at(8), matrix_TCtoI_1.at(9), matrix_TCtoI_1.at(10), matrix_TCtoI_1.at(11),
            matrix_TCtoI_1.at(12), matrix_TCtoI_1.at(13), matrix_TCtoI_1.at(14), matrix_TCtoI_1.at(15);

    // Load these into our state
    Eigen::Matrix<double, 7, 1> extrinsics_1;
    extrinsics_1.block(0, 0, 4, 1) = ov_core::rot_2_quat(T_CtoI_1.block(0, 0, 3, 3).transpose());
    extrinsics_1.block(4, 0, 3, 1) = -T_CtoI_1.block(0, 0, 3, 3).transpose() * T_CtoI_1.block(0, 3, 3, 1);



#ifdef ZED
    params.camera_wh.insert({1, {672, 376}});
#else
    params.camera_intrinsics.insert({1, std::make_shared<ov_core::CamRadtan>(752, 480)});
//    params.camera_wh.insert({1, {752, 480}});
#endif

    params.state_options.num_cameras = 2;
    params.use_stereo = false;
    if(params.state_options.num_cameras == 2) {
        //params.camera_fisheye.insert({1, false});
        params.camera_intrinsics.at(1)->set_value(intrinsics_1);
        params.camera_extrinsics.insert({1, extrinsics_1});
        params.use_stereo = true;
    }

    // params.state_options.max_slam_features = 0;
    params.init_options.init_window_time = 2;
    params.init_options.init_max_features = 75;
#ifdef ZED
    // Hand tuned
    arams.init_options.init_imu_thresh = 0.5;
#elif ANDROID_CAM
    params.init_options.init_imu_thresh = 0.0005;
#else
    // EuRoC
    params.init_options.init_imu_thresh = 1.5;
#endif
    params.fast_threshold = 20;
    params.grid_x = 5;
    params.grid_y = 5;
#ifdef ZED
    // Hand tuned
  params.num_pts = 200;
#elif ANDROID_CAM
    params.num_pts = 100;
#else
    params.num_pts = 150;
#endif
    params.msckf_options.chi2_multipler = 1;
    params.knn_ratio = .7;

    params.state_options.imu_avg = true;
    params.state_options.do_fej = true;
    params.state_options.use_rk4_integration = true;

//	params.state_options.do_calib_camera_pose = true;
//	params.state_options.do_calib_camera_intrinsics = true;
//	params.state_options.do_calib_camera_timeoffset = true;

    params.state_options.do_calib_camera_pose = false;
    params.state_options.do_calib_camera_intrinsics = false;
    params.state_options.do_calib_camera_timeoffset = false;

    params.dt_slam_delay = 3.0;
    //params.state_options.max_slam_features = 50;
    params.state_options.max_slam_features = 25;
//    params.state_options.max_slam_in_update = 25;
    params.state_options.max_slam_in_update = 25;
    //params.state_options.max_msckf_in_update = 999;
    params.state_options.max_msckf_in_update = 30;

    params.init_options.init_max_disparity = 10.0 ;//# max disparity to consider the platform stationary (dependent on resolution)
    params.init_options.init_max_features = 75 ;//# how many features to track during initialization (saves on computation)
    params.init_options.init_dyn_use = false ;// if dynamic initialization should be used
    params.init_options.init_dyn_mle_opt_calib = false ;//# if we should optimize calibration during intialization (not recommended)
    params.init_options.init_dyn_mle_max_iter = 50 ;//# how many iterations the MLE refinement should use (zero to skip the MLE)
    params.init_options.init_dyn_mle_max_time = 0.05 ;// # how many seconds the MLE should be completed in
    params.init_options.init_dyn_mle_max_threads = 6;// # how many threads the MLE should use
    params.init_options.init_dyn_num_pose = 6 ;//# number of poses to use within our window time (evenly spaced)
    params.init_options.init_dyn_min_deg = 10.0;// # orientation change needed to try to init

    params.init_options.init_dyn_inflation_orientation = 10 ;//# what to inflate the recovered q_GtoI covariance by
    params.init_options.init_dyn_inflation_velocity = 100 ;//# what to inflate the recovered v_IinG covariance by
    params.init_options.init_dyn_inflation_bias_gyro = 10;// # what to inflate the recovered bias_g covariance by
    params.init_options.init_dyn_inflation_bias_accel = 100;// # what to inflate the recovered bias_a covariance by
    params.init_options.init_dyn_min_rec_cond = 1e-12;// # reciprocal condition number thresh for info inversion

    params.init_options.init_dyn_bias_g = { 0.0, 0.0, 0.0 } ;//# initial gyroscope bias guess
    params.init_options.init_dyn_bias_a = { 0.0, 0.0, 0.0 } ;//# initial accelerometer bias guess

#ifdef ZED
    // Pixel noise; ZED works with defaults values but these may better account for rolling shutter
  params.msckf_options.chi2_multipler = 2;
  params.msckf_options.sigma_pix = 5;
	params.slam_options.chi2_multipler = 2;
	params.slam_options.sigma_pix = 5;

  // IMU biases from https://github.com/rpng/open_vins/issues/52#issuecomment-619480497
  params.imu_noises.sigma_a = 0.00395942;  // Accelerometer noise
  params.imu_noises.sigma_ab = 0.00072014; // Accelerometer random walk
  params.imu_noises.sigma_w = 0.00024213;  // Gyroscope noise
  params.imu_noises.sigma_wb = 1.9393e-05; // Gyroscope random walk
#elif ANDROID_CAM
//	params.msckf_options.chi2_multipler = 100000;
//	params.msckf_options.sigma_pix = 5;
//	params.slam_options.chi2_multipler = 100000;
//	params.slam_options.sigma_pix = 5;
    params.imu_noises.sigma_a = 16.0e-2;//0.0008413706241709801;  // Accelerometer noise
    params.imu_noises.sigma_ab = 5.5e-4;//0.00018303491436613277; // Accelerometer random walk
    params.imu_noises.sigma_w = 24.0e-3;//0.00011348681439197019;  // Gyroscope noise
    params.imu_noises.sigma_wb = 2.0e-4;//1.5495247232934592e-06; // Gyroscope random walk
#else
    params.slam_options.chi2_multipler = 1;
    params.slam_options.sigma_pix = 1;
#endif

    params.use_aruco = false;
    params.use_klt = true;

    //params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_MSCKF_INVERSE_DEPTH");
    //params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
    //params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");

    return params;
}

class slam2 : public plugin {
public:
    /**
     * @brief Constructor of the plugin that should instantiate OpenVINS
     * @param name_ Pretty name of this plugin
     * @param pb_ Phonebook that is used to connect our plugins
     */
    slam2(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , _m_rtc{pb->lookup_impl<RelativeClock>()}
            , _m_pose{sb->get_writer<pose_type>("slow_pose")}
            , ov_system{manager_params}
#if ENABLE_OPENVINS_PREDICT
            , _m_pose_fast{sb->get_writer<imu_raw_type>("imu_raw")}
#endif
            , _m_imu_integrator_input{sb->get_writer<imu_integrator_input>("imu_integrator_input")}
            , imu_cam_buffer{nullptr}
            , _m_cam_pub{sb->get_writer<cam_type>("vins")}
            , ov_update_running(false)
            , new_state_valid(false)
//            , root_path{getenv("OPENVINS_ROOT")}
//            , sensor_name{getenv("OPENVINS_SENSOR")}
            {
        // Create subscriber for our IMU
        // NOTE: ILLIXR plugins can only have a single schedule / subscriber
        // NOTE: Thus here we will subscribe directly to the IMU and use the buffer_reader interface to get camera
        sb->schedule<imu_cam_type>(id, "imu_cam", [&](switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no) {
            feed_imu_cam(datum, iteration_no);
        });

        // Load our parser and the user's verbosity
//        boost::filesystem::path config_path = root_path / "config" / sensor_name / "estimator_config.yaml";
//        auto                    parser      = std::make_shared<ov_core::YamlParser>(config_path.string());
//        std::string             verbosity   = "INFO";
//        parser->parse_config("verbosity", verbosity);
//        ov_core::Printer::setPrintLevel(verbosity);

        // Load the yaml config parameters (estimator and sensors)
//        manager_params.print_and_load(parser);
#ifndef NDEBUG
        manager_params.record_timing_information = true;
#endif /// NDEBUG
       // ov_system = std::make_shared<VioManager>(manager_params);
;
#ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{"-1"});
#endif
    }

    /**
     * @brief Callback when we have a new IMU message
     *
     * The IMU should always append to OpenVINS feed function and not be blocked by update.
     * Additionally, we publish a "fast pose" on the "imu_raw" topic for the headset.
     * If we are not processing any images currently we will create an async thread and call update.
     * After update we publish the updated pose along with the visualization image.
     *
     * @param datum IMU data packet from the switchboard
     * @param iteration_no TODO: what is this? not sure...
     */
    void feed_imu_cam(switchboard::ptr<const imu_cam_type> datum, std::size_t iteration_no) {
        // Ensures that slam doesn't start before valid IMU readings come in
        if (datum == nullptr)
            return;

        // Feed the IMU measurement. There should always be IMU data in each call to feed_imu_cam
        double           time_imu  = duration2double(datum->time.time_since_epoch());
        ov_core::ImuData imu_datum = {time_imu, datum->angular_v.cast<double>(), datum->linear_a.cast<double>()};
        ov_system.feed_measurement_imu(imu_datum);
        PRINT_ALL(BLUE "imu = %.8f (%.1f HZ)\n" RESET, imu_datum.timestamp, 1.0 / (time_imu - last_imu_time));
        last_imu_time = time_imu;

#if ENABLE_OPENVINS_PREDICT
        // Predict the state to this time (fast..)!!
        if (ov_system->initialized()) {
            std::shared_ptr<State>        state      = ov_system->get_state();
            Eigen::Matrix<double, 13, 1>  state_plus = Eigen::Matrix<double, 13, 1>::Zero();
            Eigen::Matrix<double, 12, 12> cov_plus   = Eigen::Matrix<double, 12, 12>::Zero();
            if (ov_system->get_propagator()->fast_state_propagate(state, time_imu, state_plus, cov_plus)) {
                // Velocity from fast predict are in the body frame
                // This re-rotate it into the global frame of reference v_IiinG
                Eigen::Matrix<double, 4, 1> curr_quat = state_plus.block(0, 0, 4, 1);
                Eigen::Matrix<double, 3, 1> curr_pos  = state_plus.block(4, 0, 3, 1);
                Eigen::Matrix<double, 3, 1> curr_vel =
                    ov_core::quat_2_Rot(curr_quat).transpose() * state_plus.block(7, 0, 3, 1);

                // TODO: can we get a better second IMU by storing? is this important?
                Eigen::Matrix<double, 3, 1> w_hat  = imu_datum.wm;
                Eigen::Matrix<double, 3, 1> a_hat  = imu_datum.am;
                Eigen::Matrix<double, 3, 1> w_hat2 = imu_datum.wm;
                Eigen::Matrix<double, 3, 1> a_hat2 = imu_datum.am;

                // Publish
                _m_pose_fast.put(_m_pose_fast.allocate(
                    w_hat, a_hat, w_hat2, a_hat2, curr_pos, curr_vel,
                    Eigen::Quaterniond{curr_quat(3), curr_quat(0), curr_quat(1), curr_quat(2)}, datum->time));
                // PRINT_ALL(BLUE "fast pub = %.8f\n" RESET, time_imu);
            }
        }
#endif

        // Get the async buffer next camera
//        if (_m_cam.size() == 0)
//            return;
        if (!datum->img0.has_value() && !datum->img1.has_value()) {
            return;
        } else if (imu_cam_buffer == NULL) {
            imu_cam_buffer = datum;
            return;
        }
        cv::Mat img0{imu_cam_buffer->img0.value()};
        cv::Mat img1{imu_cam_buffer->img1.value()};
        //switchboard::ptr<const cam_type> cam = _m_cam.dequeue();

#ifdef CV_HAS_METRICS
        cv::metrics::setAccount(new std::string{std::to_string(iteration_no)});
        if (iteration_no % 20 == 0) {
            cv::metrics::dump();
        }
#else
//#warning "No OpenCV metrics available. Please recompile OpenCV from git clone --branch 3.4.6-instrumented https://github.com/ILLIXR/opencv/. (see install_deps.sh)"
#endif

        // If the processing queue is currently active / running just return so we can keep getting measurements
        // Otherwise create a second thread to do our update in an async manor
        // The visualization of the state, images, and features will be synchronous with the update!
        if (ov_update_running)
            return;

        // If we have initialized then we should report the current pose
        // NOTE: This needs to be in our main thread as it seems the switchboard has mutex problems
        if (ov_system.initialized() && new_state_valid) {
            // Get the pose returned from SLAM's last update
            std::lock_guard<std::mutex> lck(new_state_mtx);
            time_point                  time_of_update(from_seconds(new_state_time));
            Eigen::Vector4d             quat   = new_state.block(0, 0, 4, 1);
            Eigen::Vector3d             pose   = new_state.block(4, 0, 3, 1);
            Eigen::Vector3d             vel    = new_state.block(7, 0, 3, 1);
            Eigen::Vector3d             bias_g = new_state.block(10, 0, 3, 1);
            Eigen::Vector3d             bias_a = new_state.block(13, 0, 3, 1);

            // OpenVINS has the rotation R_GtoIi in its state in JPL quaterion
            // We thus switch it to hamilton quaternions by simply flipping the order of xyzw
            // This results in the rotation R_IitoG being reported along with the p_IiinG (e.g. SE(3) pose)
            Eigen::Vector3f    swapped_pos = Eigen::Vector3f{float(pose(0)), float(pose(1)), float(pose(2))};
            Eigen::Quaternionf swapped_rot = Eigen::Quaternionf{float(quat(3)), float(quat(0)), float(quat(1)), float(quat(2))};
            Eigen::Quaterniond swapped_rot2 = Eigen::Quaterniond{(quat(3)), (quat(0)), (quat(1)), (quat(2))};
            assert(isfinite(swapped_rot.w()));
            assert(isfinite(swapped_rot.x()));
            assert(isfinite(swapped_rot.y()));
            assert(isfinite(swapped_rot.z()));
            assert(isfinite(swapped_pos[0]));
            assert(isfinite(swapped_pos[1]));
            assert(isfinite(swapped_pos[2]));

            // Push back this pose to our "slow" pose topic
            _m_pose.put(_m_pose.allocate(time_of_update, swapped_pos, swapped_rot));

            // Also send this information with biases and velocity to the fast IMU integrator
            imu_params params = {
                    .gyro_noise            = ov_system.get_params().imu_noises.sigma_w,
                    .acc_noise             = ov_system.get_params().imu_noises.sigma_a,
                    .gyro_walk             = ov_system.get_params().imu_noises.sigma_wb,
                    .acc_walk              = ov_system.get_params().imu_noises.sigma_ab,
                    .n_gravity             = Eigen::Matrix<double, 3, 1>(0.0, 0.0, -ov_system.get_params().gravity_mag),
                    .imu_integration_sigma = 1.0,
                    .nominal_rate          = 200.0,
            };
            _m_imu_integrator_input.put(_m_imu_integrator_input.allocate(time_of_update, from_seconds(new_state_camdt), params,
                                                                         bias_a, bias_g, pose, vel, swapped_rot2));
            new_state_valid = false;
        }

        // Check if we should drop this image
        double time_cam   = duration2double(datum->time.time_since_epoch());
        double time_delta = 1.0 / ov_system.get_params().track_frequency;
        if (time_cam < ov_update_last_timestamp + time_delta)
            return;
        ov_update_last_timestamp = time_cam;

        // Create the camera data type from switchboard data
        ov_core::CameraData cam_datum;
        cam_datum.timestamp = time_cam;
        cam_datum.sensor_ids.push_back(0);
        cam_datum.images.push_back(img0.clone());
        if (ov_system.get_params().use_mask) {
            assert(ov_system.get_params().masks.at(0).rows == img0.rows);
            assert(ov_system.get_params().masks.at(0).cols == img0.cols);
            cam_datum.masks.push_back(ov_system.get_params().masks.at(0));
        } else {
            cam_datum.masks.push_back(cv::Mat::zeros(img0.rows, img0.cols, CV_8UC1));
        }

        // If we have a stereo image we should append it too...
        if (!img1.empty() && ov_system.get_params().state_options.num_cameras > 1) {
            cam_datum.sensor_ids.push_back(1);
            cam_datum.images.push_back(img1.clone());
            if (ov_system.get_params().use_mask) {
                assert(ov_system.get_params().masks.at(1).rows == img1.rows);
                assert(ov_system.get_params().masks.at(1).cols == img1.cols);
                cam_datum.masks.push_back(ov_system.get_params().masks.at(1));
            } else {
                cam_datum.masks.push_back(cv::Mat::zeros(img1.rows, img1.cols, CV_8UC1));
            }
        }
        ov_update_camera_queue.push_back(cam_datum);
        std::sort(ov_update_camera_queue.begin(), ov_update_camera_queue.end());
        //PRINT_ALL(BLUE "camera = %.8f (%zu in queue)\n" RESET, cam_datum.timestamp, _m_cam.size());

        // Lets multi-thread it!
        ov_update_running = true;
        std::thread thread([&] {
            // Loop through our queue and see if we are able to process any of our camera measurements
            // We are able to process if we have at least one IMU measurement greater than the camera time
            double timestamp_imu_inC = time_imu - ov_system.get_state()->_calib_dt_CAMtoIMU->value()(0);
            while (!ov_update_camera_queue.empty() && ov_update_camera_queue.at(0).timestamp < timestamp_imu_inC) {
                auto   cam_datum_new = ov_update_camera_queue.at(0);
                auto   rT0_1         = boost::posix_time::microsec_clock::local_time();
                double update_dt     = 100.0 * (timestamp_imu_inC - cam_datum_new.timestamp);

                // Actually do our update!
                ov_system.feed_measurement_camera(cam_datum_new);
                PRINT_ALL(BLUE "update = %.8f (%zu in queue)\n" RESET, cam_datum_new.timestamp, ov_update_camera_queue.size());

                // Save the state if we have initialized the system
                std::lock_guard<std::mutex> lck(new_state_mtx);
                if (ov_system.initialized()) {
                    new_state       = ov_system.get_state()->_imu->value();
                    new_state_time  = ov_system.get_state()->_timestamp;
                    new_state_camdt = ov_system.get_state()->_calib_dt_CAMtoIMU->value()(0);
                    new_state_valid = true;
                }

                // Debug testing for what is the update is very very slow...
                // usleep(0.1 * 1e6);

                // Debug display of tracks
                // TODO: can this be clean up to look nicer?
                cv::Mat imgout = ov_system.get_historical_viz_image();
                if (!imgout.empty()) {
                    auto              rT0_2          = boost::posix_time::microsec_clock::local_time();
                    double            viz_track_rate = 1.0 / ((rT0_2 - rT0_1).total_microseconds() * 1e-6);
                    std::stringstream stream;
                    stream << std::fixed << std::setprecision(1) << viz_track_rate;
                    std::string rate = stream.str() + "hz";
                    cv::putText(imgout, rate, cv::Point(60, imgout.rows - 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                                cv::Scalar(0, 255, 0), 2);
                    _m_cam_pub.put(
                            _m_cam_pub.allocate<cam_type>({time_point(from_seconds(cam_datum_new.timestamp)), imgout, cv::Mat()}));
                }

                // We are done with this image!
                ov_update_camera_queue.pop_front();
                auto   rT0_2      = boost::posix_time::microsec_clock::local_time();
                double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
                PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total,
                        update_dt);
            }

            // Mark that we have finished our async processing of the image queue
            ov_update_running = false;
        });

        // We detach this thread so it runs in the background!
        // It is important that we do not lose any IMU messages...
        thread.detach();
    }

private:
    // ILLIXR glue and publishers
    const std::shared_ptr<switchboard> sb;
    std::shared_ptr<RelativeClock>     _m_rtc;
    switchboard::writer<pose_type>     _m_pose;
#if ENABLE_OPENVINS_PREDICT
    switchboard::writer<imu_raw_type> _m_pose_fast;
#endif
    VioManagerOptions manager_params = create_params();
   // std::shared_ptr<VioManager> ov_system;
    VioManager ov_system;

    switchboard::writer<imu_integrator_input> _m_imu_integrator_input;

    // Camera subscriber and tracking image publisher
//    switchboard::buffered_reader<cam_type> _m_cam;
    switchboard::ptr<const imu_cam_type> imu_cam_buffer;
    switchboard::writer<cam_type>          _m_cam_pub;

    // OpenVINS related
    std::atomic<bool>           ov_update_running;

    // Queue up camera measurements sorted by time and trigger once we have
    // exactly one IMU measurement with timestamp newer than the camera measurement
    // This also handles out-of-order camera measurements, which is rare, but
    // a nice feature to have for general robustness to bad camera drivers.
    double                          ov_update_last_timestamp = -1.0;
    std::deque<ov_core::CameraData> ov_update_camera_queue;
    double                          last_imu_time = -1.0;

    // If we have a new state this will be stored here and published by the next IMU
    // We need to do this as switchboard seems to not like async threads..
    std::atomic<bool> new_state_valid;
    std::mutex        new_state_mtx;
    Eigen::VectorXd   new_state;
    double            new_state_time;
    double            new_state_camdt;


    // How to get the configuration information
//    boost::filesystem::path root_path;
//    std::string             sensor_name;
};

PLUGIN_MAIN(slam2)
