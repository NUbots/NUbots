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
#ifndef MODULE_SKILL_QUINTICWALK_HPP
#define MODULE_SKILL_QUINTICWALK_HPP

#include <map>
#include <memory>
#include <nuclear>
#include <vector>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/KinematicsModel.hpp"
#include "message/actuation/Servos.hpp"
#include "message/behaviour/state/Stability.hpp"

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
            double max_step_xy       = 0.0f;

            bool imu_active            = true;
            double imu_pitch_threshold = 0.0f;
            double imu_roll_threshold  = 0.0f;

            utility::skill::WalkingParameter params{};

            std::map<message::actuation::ServoID, message::actuation::ServoState> servo_states{};

            std::vector<std::pair<message::actuation::ServoID, double>> arm_positions{};
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
