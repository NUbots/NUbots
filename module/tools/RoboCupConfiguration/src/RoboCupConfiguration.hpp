/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
#define MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP

#include <Eigen/Core>
#include <nuclear>
#include <string>

namespace module::tools {

    class RoboCupConfiguration : public NUClear::Reactor {
    private:
        std::string hostname           = "";
        std::string wifi_interface     = "";
        std::string ip_address         = "";
        std::string ssid               = "";
        std::string password           = "";
        int team_id                    = 0;
        int player_id                  = 0;
        Eigen::Vector3d ready_position = Eigen::Vector3d::Zero();

        /// @brief Smart enum for the robot's position
        struct Position {
            enum Value {
                STRIKER,
                GOALIE,
                DEFENDER,
            };
            Value value = Value::STRIKER;

            Position() = default;
            Position(std::string const& str) {
                // clang-format off
                if (str == "STRIKER") { value = Value::STRIKER; }
                else if (str == "GOALIE") { value = Value::GOALIE; }
                else if (str == "DEFENDER") { value = Value::DEFENDER; }
                else { throw std::runtime_error("Invalid robot position"); }
                // clang-format on
            }

            operator std::string() const {
                switch (value) {
                    case Value::STRIKER: return "STRIKER";
                    case Value::DEFENDER: return "DEFENDER";
                    case Value::GOALIE: return "GOALIE";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            std::string get_config_name() {
                switch (value) {
                    case Value::STRIKER: return "Striker.yaml";
                    case Value::DEFENDER: return "Defender.yaml";
                    case Value::GOALIE: return "Goalie.yaml";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            operator int() const {
                return value;
            }

            void operator++() {
                value =
                    value == Value::DEFENDER ? Value::STRIKER : (value == Value::STRIKER ? Value::GOALIE : DEFENDER);
            }
        } robot_position;

        /// @brief The index of the item we are selecting
        size_t row_selection = 0;

        /// @brief Index of the column we are selecting
        size_t column_selection = 0;

        std::string log_message = "";

        void refresh_view();
        void edit_selection();
        void toggle_selection();
        std::string user_input();
        void get_values();
        void set_values();
        std::string get_platform();
        std::string get_config_file(std::string filename);
        void configure_network();
        std::string get_wifi_password(const std::string& ssid);

    public:
        /// @brief Called by the powerplant to build and setup the RoboCupConfiguration reactor.
        explicit RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
