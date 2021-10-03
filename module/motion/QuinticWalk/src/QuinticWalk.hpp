#ifndef MODULE_MOTION_QUINTICWALK_HPP
#define MODULE_MOTION_QUINTICWALK_HPP

#include <map>
#include <memory>
#include <nuclear>
#include <vector>

#include "WalkEngine.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/ServoCommand.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/input/ServoID.hpp"

namespace module::motion {

    class QuinticWalk : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the QuinticWalk reactor.
        explicit QuinticWalk(std::unique_ptr<NUClear::Environment> environment);

        static constexpr int UPDATE_FREQUENCY = 200;

    private:
        /// Current subsumption ID key to access motors.
        size_t subsumptionId = 1;

        // Reaction handle for the main update loop, disabling when not moving will save unnecessary CPU
        ReactionHandle update_handle{};
        ReactionHandle imu_reaction{};

        void calculateJointGoals();
        [[nodiscard]] float getTimeDelta();
        [[nodiscard]] std::unique_ptr<message::behaviour::ServoCommands> motion(
            const std::vector<std::pair<utility::input::ServoID, float>>& joints);

        struct Config {
            Eigen::Vector3f max_step = Eigen::Vector3f::Zero();
            float max_step_xy        = 0.0f;

            bool imu_active           = true;
            float imu_pitch_threshold = 0.0f;
            float imu_roll_threshold  = 0.0f;

            WalkingParameter params{};

            std::map<utility::input::ServoID, float> jointGains{};
            std::vector<std::pair<utility::input::ServoID, float>> arm_positions{};
        } normal_config{}, goalie_config{};

        static void load_quintic_walk(const ::extension::Configuration& cfg, Config& config);

        Config& current_config = normal_config;
        bool first_config      = true;

        Eigen::Vector3f current_orders = Eigen::Vector3f::Zero();
        bool is_left_support           = true;
        bool falling                   = false;
        bool first_run                 = true;

        NUClear::clock::time_point last_update_time{};

        QuinticWalkEngine walk_engine{};

        message::motion::KinematicsModel kinematicsModel{};

        Eigen::Vector3f trunk_pos  = Eigen::Vector3f::Zero();
        Eigen::Vector3f trunk_axis = Eigen::Vector3f::Zero();
        Eigen::Vector3f foot_pos   = Eigen::Vector3f::Zero();
        Eigen::Vector3f foot_axis  = Eigen::Vector3f::Zero();
    };
}  // namespace module::motion

#endif  // MODULE_MOTION_QUINTICWALK_HPP
