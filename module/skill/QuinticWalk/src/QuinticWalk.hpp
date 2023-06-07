#ifndef MODULE_SKILL_QUINTICWALK_HPP
#define MODULE_SKILL_QUINTICWALK_HPP

#include <map>
#include <memory>
#include <nuclear>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/state/Stability.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/skill/WalkEngine.hpp"

namespace module::skill {

    class QuinticWalk : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the QuinticWalk reactor.
        explicit QuinticWalk(std::unique_ptr<NUClear::Environment> environment);

        static constexpr int UPDATE_FREQUENCY = 200;

    private:
        // Reaction handle for the imu reaction, disabling when not moving will save unnecessary CPU
        ReactionHandle imu_reaction{};

        void calculate_joint_goals();
        [[nodiscard]] double get_time_delta();

        struct Config {
            Eigen::Vector3d max_step = Eigen::Vector3d::Zero();
            double max_step_xy       = 0.0;

            bool imu_active            = true;
            double imu_pitch_threshold = 0.0;
            double imu_roll_threshold  = 0.0;

            utility::skill::WalkingParameter params{};

            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states{};

            std::vector<std::pair<utility::input::ServoID, double>> arm_positions{};
        } normal_cfg{}, goalie_cfg{};

        static void load_quintic_walk(const ::extension::Configuration& cfg, Config& config);

        /// @brief Stores the walking config
        Config& current_cfg = normal_cfg;

        bool first_cfg = true;

        Eigen::Vector3d current_orders = Eigen::Vector3d::Zero();
        bool is_left_support           = true;
        bool first_run                 = true;

        NUClear::clock::time_point last_update_time{};

        utility::skill::QuinticWalkEngine walk_engine{};

        message::actuation::KinematicsModel kinematicsModel{};
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_QUINTICWALK_HPP
