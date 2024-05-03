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
        /// @brief The hostname of the robot
        std::string hostname = "";
        /// @brief The wifi interface that the robot is connected to
        std::string wifi_interface = "";
        /// @brief The IP address of the robot
        std::string ip_address = "";
        /// @brief The SSID of the wifi network that the robot is or will be connected to
        std::string ssid = "";
        /// @brief The password of the wifi network that the robot is or will be connected to
        std::string password = "";
        /// @brief The ID that the robot is player for
        int team_id = 0;
        /// @brief The player ID of the robot
        int player_id = 0;
        /// @brief The position the robot will move to in READY
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

            /// @brief Convert the enum to a string
            operator std::string() const {
                switch (value) {
                    case Value::STRIKER: return "STRIKER";
                    case Value::DEFENDER: return "DEFENDER";
                    case Value::GOALIE: return "GOALIE";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            /// @brief Get the full yaml name of the config for the current position
            /// @return The full yaml name of the config for the current position
            std::string get_config_name() {
                switch (value) {
                    case Value::STRIKER: return "Striker.yaml";
                    case Value::DEFENDER: return "Defender.yaml";
                    case Value::GOALIE: return "Goalie.yaml";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            /// @brief Increment the enum, for toggle
            void operator++() {
                value =
                    value == Value::DEFENDER ? Value::STRIKER : (value == Value::STRIKER ? Value::GOALIE : DEFENDER);
            }
        } robot_position;

        /// @brief The index of the item the user is selecting
        size_t row_selection = 0;

        /// @brief Index of the column the user is selecting
        size_t column_selection = 0;

        /// @brief The log message to print to the user at the bottom of the window
        std::string log_message = "";

        /// @brief Displays the screen with any updated values to the user
        void refresh_view();

        /// @brief Functionality for the user to edit a field in the display
        void edit_selection();

        /// @brief Functionality for the user to toggle fields in the display
        void toggle_selection();

        /// @brief Used in edit_selection to get user input
        /// @return The user input
        std::string user_input();

        /// @brief Gets the current configurable values to display to the user
        void get_config_values();

        /// @brief Sets the values set by the user (ie what is shown in the display) to configuration files
        void set_config_values();

        /// @brief Using the current configuration, sets the network settings.
        /// This needs sudo to run, as it edits the network configuration files.
        /// set_values() is called first in this function.
        void configure_network();

        /// @brief Gets the platform we are currently running on, eg nugus, docker, webots, from the hostname
        /// @return The platform we are currently running on
        std::string get_platform();

        /// @brief Gets the configuration file according to the config system's rules.
        /// The order is: hostname-specific, platform-specific, and finally the default config.
        /// @param filename The config file to find
        /// @return The correct config file path according to config rules
        std::string get_config_file(std::string filename);


    public:
        /// @brief Called by the powerplant to build and setup the RoboCupConfiguration reactor.
        explicit RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
