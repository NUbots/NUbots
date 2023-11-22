/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_SKILL_WALK_HPP
#define MODULE_SKILL_WALK_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/math/control/pid.hpp"
#include "utility/math/filter/ExponentialFilter.hpp"
#include "utility/skill/WalkGenerator.hpp"

namespace module::skill {

    using utility::math::control::PID;
    using utility::math::filter::ExponentialFilter;
    using utility::skill::WalkGenerator;

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
        WalkGenerator<double> walk_generator{};

        /// @brief Torso X-Y position PID controller
        PID<double, 2> torso_controller{};

        /// @brief Exponential filter for torso X-Y position control input (position offset)
        ExponentialFilter<double, 2> torso_controller_filter{};

        /// @brief Torso pitch PID controller
        utility::math::control::PID<double, 1> pitch_controller{};

        /// @brief Compute torso position offset to keep the robot balanced
        Eigen::Isometry3d compute_torso_offset(const Eigen::Isometry3d& Htp_desired,
                                               const Eigen::Isometry3d& Htp_measured,
                                               const double time_delta);
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
