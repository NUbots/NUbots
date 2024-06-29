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
#ifndef MODULE_PURPOSE_TESTER_HPP
#define MODULE_PURPOSE_TESTER_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    struct StartTester {};

    class Tester : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Priority of StartSafely task
            int start_safely_priority = 0;
            /// @brief Priority of FindBall task
            int find_ball_priority = 0;
            /// @brief Priority of LookAtBall task
            int look_at_ball_priority = 0;
            /// @brief Priority of StandStill task
            int walk_to_ball_priority = 0;
            /// @brief Priority of StandStill task
            int walk_to_kick_ball_priority = 0;
            /// @brief Priority of AlignBallToGoal task
            int align_ball_to_goal_priority = 0;
            /// @brief Priority of KickToGoal task
            int kick_to_goal_priority = 0;
            /// @brief Priority of WalkToFieldPosition task
            int walk_to_field_position_priority = 0;
            /// @brief Priority of KickTo task
            int kick_to_priority = 0;
            /// @brief Priority of LookAround task
            int look_around_priority = 0;
            /// @brief Priority of StandStill task
            int stand_still_priority = 0;
            /// @brief Priority of Say task
            int say_priority = 0;
            /// @brief Priority of ChatGPT task
            int chatgpt_priority = 0;
            /// @brief Priority of AudioGPT task
            int audiogpt_priority = 0;
            /// @brief Position to walk to when emitting WalkToFieldPosition task
            Eigen::Vector3f walk_to_field_position_position = Eigen::Vector3f::Zero();
            /// @brief Text to say when emitting Say task
            std::string say_text = "";
            /// @brief Text to prompt ChatGPT with
            std::string chatgpt_prompt = "";
            /// @brief Duration to listen for audio when emitting AudioGPT task
            int audiogpt_listen_duration = 0;
            /// @brief Delay in seconds before creating director tree
            int start_delay = 0;
        } cfg;

        /// @brief The rate the tasks will emit, to drive the rest of the system
        static constexpr size_t BEHAVIOUR_UPDATE_RATE = 10;

    public:
        /// @brief Called by the powerplant to build and setup the Tester reactor.
        explicit Tester(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_TESTER_HPP
