#ifndef MODULE_SKILL_WALK_HPP
#define MODULE_SKILL_WALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/skill/WalkGenerator.hpp"

namespace module::skill {

    class Walk : public ::extension::behaviour::BehaviourReactor {

    public:
        /// @brief Called by the powerplant to build and setup the Walk reactor
        explicit Walk(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Frequency of walk engine updates
        static constexpr int UPDATE_FREQUENCY = 200;

    private:
        struct Config {
            /// @brief Stores the gains for each servo
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states{};

            /// @brief Desired arm positions while walking
            std::vector<std::pair<utility::input::ServoID, double>> arm_positions{};

            /// @brief Walk engine parameters
            utility::skill::WalkParameters<double> walk_generator_parameters{};

            /// @brief P gain for the leg servos
            double leg_servo_gain = 0.0;

            /// @brief P gain for the arm servos
            double arm_servo_gain = 0.0;

            /// @brief Torso position controller P gain
            double K_torso = 0.0;

            /// @brief Torso position controller I gain
            double Ki_torso = 0.0;
        } cfg;

        /// @brief Last time we updated the walk engine
        NUClear::clock::time_point last_update_time{};

        /// @brief Generates swing foot and torso trajectories for given walk velocity target
        utility::skill::WalkGenerator<double> walk_generator{};

        /// @brief Torso X-Y offset computed from the PI controller
        Eigen::Vector2d torso_offset{};
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
