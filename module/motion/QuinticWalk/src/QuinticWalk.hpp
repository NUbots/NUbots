#ifndef MODULE_MOTION_QUINTICWALK_H
#define MODULE_MOTION_QUINTICWALK_H

#include <map>
#include <memory>
#include <nuclear>
#include <vector>

#include "WalkEngine.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/ServoID.hpp"

namespace module {
namespace motion {

    class QuinticWalk : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the QuinticWalk reactor.
        explicit QuinticWalk(std::unique_ptr<NUClear::Environment> environment);

        static constexpr int UPDATE_FREQUENCY = 200;

    private:
        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
        ReactionHandle update_handle;
        ReactionHandle imu_reaction;

        void calculateJointGoals();
        float getTimeDelta();
        std::unique_ptr<std::vector<message::behaviour::ServoCommand>> motionLegs(
            const std::vector<std::pair<utility::input::ServoID, float>>& joints);

        struct {
            Eigen::Vector3f max_step;
            float max_step_xy;

            bool imu_active;
            float imu_pitch_threshold;
            float imu_roll_threshold;
        } config;

        Eigen::Vector3f current_orders;
        bool is_left_support;
        bool falling;
        bool first_run;

        NUClear::clock::time_point last_update_time;

        QuinticWalkEngine walk_engine;
        WalkingParameter params;

        message::motion::KinematicsModel kinematicsModel;

        Eigen::Vector3f trunk_pos;
        Eigen::Vector3f trunk_axis;
        Eigen::Vector3f foot_pos;
        Eigen::Vector3f foot_axis;

        std::map<utility::input::ServoID, float> jointGains;
    };

}  // namespace motion
}  // namespace module

#endif
