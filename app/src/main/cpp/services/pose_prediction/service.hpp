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
    const std::shared_ptr<const relative_clock> clock_;
    switchboard::reader<data_format::pose_type> slow_pose_;
    switchboard::reader<data_format::imu_raw_type> imu_raw_;
    switchboard::reader<data_format::pose_type> true_pose_;
    switchboard::reader<switchboard::event_wrapper<Eigen::Vector3f>> ground_truth_offset_;
    switchboard::reader<switchboard::event_wrapper<time_point>> vsync_estimate_;
    mutable Eigen::Quaternionf offset_{Eigen::Quaternionf::Identity()};
    mutable std::shared_mutex offset_mutex_;

};

} // namespace ILLIXR

}