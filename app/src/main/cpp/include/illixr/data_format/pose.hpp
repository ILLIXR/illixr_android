#pragma once

#include "illixr/switchboard.hpp"

#include <Eigen/Dense>

namespace ILLIXR::data_format {
    struct pose_type : public switchboard::event {
        time_point         sensor_time; // Recorded time of sensor data ingestion
        Eigen::Vector3f    position;
        Eigen::Quaternionf orientation;

        pose_type()
                : sensor_time{time_point{}}
                , position{Eigen::Vector3f{0, 0, 0}}
                , orientation{Eigen::Quaternionf{1, 0, 0, 0}}
        { }
        pose_type(time_point sensor_time_,
                  Eigen::Vector3f position_,
                  Eigen::Quaternionf orientation_)
                : sensor_time{sensor_time_}
                , position{position_}
                , orientation{orientation_}
        { }
    };

    typedef struct {
        pose_type  pose;
        time_point predict_computed_time; // Time at which the prediction was computed
        time_point predict_target_time;   // Time that prediction targeted.
    } fast_pose_type;

    struct texture_pose : public switchboard::event {
        duration           offload_duration;
        unsigned char*     image;
        time_point         pose_time;
        Eigen::Vector3f    position;
        Eigen::Quaternionf latest_quaternion;
        Eigen::Quaternionf render_quaternion;

        texture_pose() { }

        texture_pose(duration offload_duration_, unsigned char* image_, time_point pose_time_, Eigen::Vector3f position_,
                     Eigen::Quaternionf latest_quaternion_, Eigen::Quaternionf render_quaternion_)
                : offload_duration{offload_duration_}
                , image{image_}
                , pose_time{pose_time_}
                , position{position_}
                , latest_quaternion{latest_quaternion_}
                , render_quaternion{render_quaternion_} { }
    };

}