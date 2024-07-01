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
#include "message/input/Purposes.hpp"

namespace module::purpose {

    using message::input::RoboCup;
    using message::input::SoccerPosition;

    class Soccer : public ::extension::behaviour::BehaviourReactor {
    private:
        uint player_id = 0;

        NUClear::clock::time_point startup_time = NUClear::clock::now();

        /// @brief Smart enum for the robot's position
        struct Position {
            enum Value {
                STRIKER,
                GOALIE,
                DEFENDER,
                DYNAMIC
            };
            Value value = Value::STRIKER;

            Position() = default;
            Position(std::string const& str) {
                // clang-format off
                if (str == "STRIKER") { value = Value::STRIKER; }
                else if (str == "GOALIE") { value = Value::GOALIE; }
                else if (str == "DEFENDER") { value = Value::DEFENDER; }
                else if (str == "DYNAMIC") { value = Value::DYNAMIC; }
                else { throw std::runtime_error("Invalid robot position"); }
                // clang-format on
            }

            SoccerPosition toSoccerPosition() const {
                switch (value) {
                    case Value::STRIKER:
                        return SoccerPosition::STRIKER;
                    case Value::GOALIE:
                        return SoccerPosition::GOALIE;
                    case Value::DEFENDER:
                        return SoccerPosition::DEFENDER;
                    default:
                        throw std::runtime_error("Invalid Position conversion");
                }
            }

            operator int() const {
                return value;
            }
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether or not to force the robot to ignore GameController and play
            bool force_playing = false;
            /// @brief The soccer position of the robot
            Position position{};
            /// @brief The number of seconds to wait before assuming a teammate is inactive
            uint8_t timeout = 0;
        } cfg;

        struct RobotInfo {
            uint8_t robot_id                            = 0;
            NUClear::clock::time_point startup_time     = NUClear::clock::now();
            NUClear::clock::time_point last_update_time = NUClear::clock::now();
            Position position                           = Position("DEFENDER");

            bool operator<(const RobotInfo& other) const {
                if (startup_time == other.startup_time) {
                    return robot_id < other.robot_id;
                }
                return startup_time < other.startup_time;
            }
        };

        /// @brief Store robots that can currently play
        std::vector<RobotInfo> active_robots{};

        /// @brief Store's the robot's current soccer position
        Position soccer_position;

        /// @brief Utility to set up robocup message to find purpose
        void find_purpose_message();

        /// @brief Add and update active robots
        /// @param robocup A robocup message sent by another robot
        void manage_active_robots(const RoboCup& robocup);

        /// @brief Add RobotInfo ordered by id to active_robots
        /// @param new_robot The new robot's information, to be added to active_robots
        void add_robot(RobotInfo new_robot);

        /// @brief Return the index of the future striker
        uint8_t find_striker();

        /// @brief Decide the correct soccer positions
        void give_directions();

        /// @brief Emit purpose based on leader's instructions
        /// @param robocup A robocup message sent by another robot
        /// @param incoming_robot_id The id of the robot that send the robocup message
        void follow_directions(const RoboCup& robocup, const uint8_t incoming_robot_id);

        /// @brief The rate the find purpose provider will run, to drive the rest of the system
        static constexpr size_t BEHAVIOUR_UPDATE_RATE = 10;

    public:
        /// @brief Called by the powerplant to build and setup the Soccer reactor.
        explicit Soccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SOCCER_HPP
