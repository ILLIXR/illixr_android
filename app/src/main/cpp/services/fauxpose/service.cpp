/*****************************************************************************/
/*                                                                           */
/* Created: 03/03/2022                                                       */
/* Last Edited: 08/06/2022                                                   */
/*                                                                           */
/* An IlliXR plugin that publishes position tracking data ("pose")           */
/*	 from a mathematical operation just to quickly produce some known    */
/*	 tracking path for the purpose of debugging other portions of IlliXR.*/
/*                                                                           */
/* TODO:                                                                     */
/*   * DONE: Need to implement as a "pose_prediction" "impl" (service?)      */
/*   * DONE: Fix so that "gldemo" (etc.) face forward.                       */
/*                                                                           */
/* NOTES:                                                                    */
/*   * get_fast_pose() method returns a "fast_pose_type"                     */
/*   * "fast_pose_type" is a "pose_type" plus computed & target timestamps   */
/*   * correct_pose() method returns a "pose_type"                           */
/*   * (This version uploaded to ILLIXR github)                              */
/*                                                                           */

#include "service.hpp"

#include "illixr/threadloop.hpp"


using namespace ILLIXR;

faux_pose_impl::faux_pose_impl(const phonebook* const pb)
            : switchboard_{pb->lookup_impl<switchboard>()} {
#ifndef NDEBUG
        std::cout << "[fauxpose] Starting Service\n";
#endif
        spdlog::get("illixr")->debug("Started faux_pose");
        // Store the initial time
        auto now       = std::chrono::system_clock::now();
    sim_start_time_ = (time_point)std::chrono::time_point_cast<std::chrono::microseconds>(now);

        // Set an initial faux-pose location
        // simulated_location = Eigen::Vector3f{0.0, 1.5, 0.0};
        period_    = 0.5;
    amplitude_ = 2.0;
    }

faux_pose_impl::~faux_pose_impl() {
#ifndef NDEBUG
        std::cout << "[fauxpose] Ending Service\n";
#endif
    }

    data_format::pose_type faux_pose_impl::get_true_pose() const {
        throw std::logic_error{"Not Implemented"};
    }

    data_format::pose_type faux_pose_impl::correct_pose([[maybe_unused]] const data_format::pose_type pose) const {
        data_format::pose_type simulated_pose;
#ifndef NDEBUG
        std::cout << "[fauxpose] Returning (zero) pose\n";
#endif
        return simulated_pose;
    }

   void faux_pose_impl::set_offset(const Eigen::Quaternionf& raw_o_times_offset) {
        std::unique_lock   lock{offset_mutex_};
        Eigen::Quaternionf raw_o = raw_o_times_offset * offset_.inverse();
        offset_                   = raw_o.inverse();
    }

    data_format::fast_pose_type faux_pose_impl::get_fast_pose() const {
        // MHuzai:  In actual pose prediction, the semantics are that
        //  we return the pose for next vsync, not now. I think we
        //  should do the same here, unless your intent is different
        //  with faux_pose.
        return get_fast_pose(std::chrono::system_clock::now());
    }

    data_format::fast_pose_type faux_pose_impl::get_fast_pose(time_point time) const {
        data_format::pose_type simulated_pose; /* The algorithmically calculated 6-DOF pose */
        double    sim_time;       /* sim_time is used to regulate a consistent movement */

        RAC_ERRNO_MSG("[fauxpose] at start of _p_one_iteration");

        // Calculate simulation time from start of execution
        std::chrono::nanoseconds elapsed_time;
        elapsed_time = time - sim_start_time_;
        sim_time     = elapsed_time.count() * 0.000000001;

        // Calculate new pose values
        //   Pose values are calculated from the passage of time to maintain consistency */
        simulated_pose.position[0] = amplitude_ * sin(sim_time * period_);     // X
        simulated_pose.position[1] = 1.5;                                    // Y
        simulated_pose.position[2] = amplitude_ * cos(sim_time * period_);     // Z
        simulated_pose.orientation = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0); // (W,X,Y,Z) Facing forward

        // Return the new pose
#ifndef NDEBUG
        std::cout << "[fauxpose] Returning pose\n";
#endif
        return data_format::fast_pose_type{
                .pose = simulated_pose, .predict_computed_time = std::chrono::system_clock::now(), .predict_target_time = time};
    }

class fauxpose : public plugin {
public:
    // ********************************************************************
    /* Constructor: Provide handles to faux_pose */
    [[maybe_unused]] fauxpose(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        // "pose_prediction" is a class inheriting from "phonebook::service"
        //   It is described in "pose_prediction.hpp"
        spdlog::get("illixr")->debug("Started faux_pose ... actual");

        pb->register_impl<data_format::pose_prediction>(std::static_pointer_cast<data_format::pose_prediction>(std::make_shared<faux_pose_impl>(pb)));
#ifndef NDEBUG
        printf("[fauxpose] Starting Plugin\n");
#endif
    }

    // ********************************************************************
    ~fauxpose() override{
#ifndef NDEBUG
        std::cout << "[fauxpose] Ending Plugin\n";
#endif
        spdlog::get("illixr")->debug("end faux_pose ... actual");

    }
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(fauxpose)
