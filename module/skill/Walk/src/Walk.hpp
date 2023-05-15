#ifndef MODULE_SKILL_WALK_HPP
#define MODULE_SKILL_WALK_HPP

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
#include "utility/skill/MotionGeneration.hpp"

namespace module::skill {

    class Walk : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the Walk reactor.
        explicit Walk(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 200;

    private:
        /// @brief Emits task to update desired joint positions to achieve desired pose from walk engine
        void update_desired_pose();

        /// @brief Computes the time delta since the last time we updated the desired joint positions
        [[nodiscard]] float get_time_delta();

        struct Config {
            /// @brief Stores the gains each servo (for joint position requests)
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states{};

            /// @brief Desired arm positions for walking
            std::vector<std::pair<utility::input::ServoID, float>> arm_positions{};
        } cfg;

        /// @brief Whether or not this is the first time we have run a desired joint position update
        bool first_run = true;

        /// @brief Last time we updated the desired joint positions
        NUClear::clock::time_point last_update_time{};

        /// @brief Walk engine, generates swing foot and torso trajectories for walking
        utility::skill::MotionGeneration<float> walk_engine{};
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
