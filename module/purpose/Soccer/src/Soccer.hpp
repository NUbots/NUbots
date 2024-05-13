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

    class Soccer : public ::extension::behaviour::BehaviourReactor {
    private:
        uint PLAYER_ID;

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

            operator int() const {
                return value;
            }

            // TODO: temp, remove me
            std::string toString() const {
                switch(value) {
                    case STRIKER: return "STRIKER";
                    case GOALIE: return "GOALIE";
                    case DEFENDER: return "DEFENDER";
                    case DYNAMIC: return "DYNAMIC";
                    default: throw std::runtime_error("Invalid value for Position");
                }
            }
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Whether or not to force the robot to ignore GameController and play
            bool force_playing = false;
            /// @brief The soccer position of the robot
            Position position{};
        } cfg;

        struct RobotInfo {
            uint8_t robot_id;
            std::chrono::steady_clock::time_point last_heard_from;
            Position position;

            bool operator<(const RobotInfo& other) const {
                return robot_id < other.robot_id;
            }
        };

        /// @brief Store and remove active/inactive robots
        bool manage_active_robots(const uint8_t robot_id);

        /// @brief Add RobotInfo ordered by id
        void add_robot(RobotInfo new_robot);

        /// @brief Count the number of defenders
        uint8_t count_defenders();

        /// @brief Decide the correct soccer position
        void find_soccer_position(const message::input::RoboCup& robocup);

        /// @brief Store robots that can currently play
        std::vector<RobotInfo> active_robots;

        /// @brief Store penalised robots
        std::list<RobotInfo> penalised_robots;

    public:
        /// @brief Called by the powerplant to build and setup the Soccer reactor.
        explicit Soccer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SOCCER_HPP
