#ifndef MODULE_SKILL_WALK_HPP
#define MODULE_SKILL_WALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/control/pid.hpp"
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

            /// @brief Desired torso pitch
            Eigen::Matrix<double, 1, 1> desired_torso_pitch = Eigen::Matrix<double, 1, 1>::Zero();

            /// @brief Torso position controller PID gains
            Eigen::Vector3d torso_pid_gains = Eigen::Vector3d::Zero();

            /// @brief Torso anti-windup limits
            Eigen::Vector2d torso_antiwindup = Eigen::Vector2d::Zero();

            /// @brief Torso pitch controller PID gains
            Eigen::Vector3d pitch_pid_gains = Eigen::Vector3d::Zero();

            /// @brief Torso pitch anti-windup limits
            Eigen::Vector2d pitch_antiwindup = Eigen::Vector2d::Zero();
        } cfg;

        /// @brief Last time we updated the walk engine
        NUClear::clock::time_point last_update_time{};

        /// @brief Generates swing foot and torso trajectories for given walk velocity target
        utility::skill::WalkGenerator<double> walk_generator{};

        /// @brief Torso X-Y position PID controller
        utility::math::control::PID<double, 2> torso_controller{};

        /// @brief Torso pitch PID controller
        utility::math::control::PID<double, 1> pitch_controller{};
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
