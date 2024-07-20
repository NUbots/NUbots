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
            utility::skill::WalkGenerator<double>::WalkParameters walk_generator_parameters{};

            /// @brief Pitch controller gain
            double pitch_gain{};

            /// @brief Pitch offset maximum
            double pitch_max_offset{};

            /// @brief Pitch controller enabled
            bool pitch_controller_enabled{};

            /// @brief Smoothing factor for velocity target
            double smoothing_factor{};
        } cfg;

        /// @brief Last time we updated the walk engine
        NUClear::clock::time_point last_update_time{};

        /// @brief Generates swing foot and torso trajectories for given walk velocity target
        utility::skill::WalkGenerator<double> walk_generator{};

        /// @brief Pitch offset
        double pitch_offset{};

        /// @brief Pitch controller reaction
        ReactionHandle pitch_controller_reaction{};

        /// @brief Current velocity target
        Eigen::Vector3d velocity_target = Eigen::Vector3d::Zero();
    };
}  // namespace module::skill

#endif  // MODULE_SKILL_WALK_HPP
