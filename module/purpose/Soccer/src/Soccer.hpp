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
#ifndef MODULE_PURPOSE_SOCCER_HPP
#define MODULE_PURPOSE_SOCCER_HPP

#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "message/input/RoboCup.hpp"

namespace module::purpose {

    using message::input::RoboCup;

    struct EnableIdle {};
    struct DisableIdle {};

    class Soccer : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief The id of the robot
        uint player_id = 0;

        /// @brief Smart enum for the robot's position
        struct Position {
            enum Value { ALL_ROUNDER, STRIKER, GOALIE, DEFENDER, DYNAMIC };
            Value value = Value::ALL_ROUNDER;

            Position() = default;
            Position(Value value) : value(value) {}
            Position(std::string const& str) {
                // clang-format off
                if (str == "ALL_ROUNDER") { value = Value::ALL_ROUNDER; }
                else if (str == "STRIKER") { value = Value::STRIKER; }
                else if (str == "GOALIE") { value = Value::GOALIE; }
                else if (str == "DEFENDER") { value = Value::DEFENDER; }
                else if (str == "DYNAMIC") { value = Value::DYNAMIC; }
                else { throw std::runtime_error("Invalid robot position"); }
                // clang-format on
            }

            operator int() const {
                return value;
            }

            operator std::string() const {
                switch (value) {
                    case Value::ALL_ROUNDER: return "AllRounder";
                    case Value::STRIKER: return "Striker";
                    case Value::GOALIE: return "Goalie";
                    case Value::DEFENDER: return "Defender";
                    case Value::DYNAMIC: return "Dynamic";
                    default: return "Unknown";
                }
            }
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether or not to force the robot to ignore GameController and play
            bool force_playing = false;
            /// @brief Delay in seconds before the robot starts playing after button press
            int disable_idle_delay = 0;
            /// @brief The soccer position of the robot
            Position position{};
            /// @brief The number of seconds to wait before assuming a teammate is inactive
            int timeout = 0;
            /// @brief The largest id of a robot
            uint8_t max_robots = 0;
        } cfg;

        struct RobotInfo {
            /// @brief If this robot has been heard within the timeout
            bool active = false;
            /// @brief The time when we last heard this robot
            NUClear::clock::time_point last_heard = NUClear::clock::now();
            /// @brief Whether or not this robot is deciding what position its in dynamically
            bool dynamic = false;
            /// @brief The current position of this robot
            Position position = Position::DYNAMIC;
            /// @brief The distance of the robot to the ball
            double distance_to_ball = std::numeric_limits<double>::max();
        };

        /// @brief Store robots that could possibly play
        std::vector<RobotInfo> robots{};

        /// @brief The rate the find purpose provider will run, to drive the rest of the system
        static constexpr size_t BEHAVIOUR_UPDATE_RATE = 10;

        /// @brief Idle state of the robot
        bool idle = false;

        /// @brief Determines what purpose the robot shall play as and emits the Task for that purpose
        void determine_purpose();

    public:
        /// @brief Called by the powerplant to build and setup the Soccer reactor.
        explicit Soccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SOCCER_HPP
