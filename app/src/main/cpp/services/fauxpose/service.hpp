#pragma once

#include "illixr/data_format/pose_prediction.hpp"
#include "illixr/switchboard.hpp"

namespace ILLIXR {
class faux_pose_impl : public data_format::pose_prediction {
public:
    // ********************************************************************
    /* Constructor: Provide handles to faux_pose */
    faux_pose_impl(const phonebook* const pb);

    ~faux_pose_impl();

    data_format::pose_type get_true_pose() const override;

    bool fast_pose_reliable() const override {
        return true;
    }

    bool true_pose_reliable() const override {
        return false;
    }

    data_format::pose_type
    correct_pose([[maybe_unused]] const data_format::pose_type pose) const override;

    Eigen::Quaternionf get_offset() override {
        return offset_;
    }

    void set_offset(const Eigen::Quaternionf& raw_o_times_offset) override;

    data_format::fast_pose_type get_fast_pose() const override;

    // ********************************************************************
    // get_fast_pose(): returns a "fast_pose_type" with the algorithmically
    //   determined location values.  (Presently moving in a circle, but
    //   always facing "front".)
    //
    // NOTE: time_type == std::chrono::system_clock::time_point
    data_format::fast_pose_type get_fast_pose(time_point time) const override;

private:
    const std::shared_ptr<switchboard> switchboard_;
    mutable Eigen::Quaternionf offset_{Eigen::Quaternionf::Identity()};
    mutable std::shared_mutex offset_mutex_;

    time_point sim_start_time_; /* Store the initial time to calculate a known runtime */
    double period_;             /* The period of the circular movment (in seconds) */
    double amplitude_;          /* The amplitude of the circular movment (in meters) */
};


}