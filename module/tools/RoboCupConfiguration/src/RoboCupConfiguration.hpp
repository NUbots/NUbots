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
        /// @brief The name of the robot
        std::string robot_name = "";
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
        /// @brief Max team team ID
        static const int MAX_TEAM_ID = 33;
        /// @brief The player ID of the robot
        int player_id = 0;
        /// @brief Max player ID
        static const int MAX_PLAYER_ID = 6;

        /// @brief Smart enum for the robot's position
        struct Position {
            enum Value { ALL_ROUNDER, STRIKER, GOALIE, DEFENDER, DYNAMIC };
            Value value = Value::ALL_ROUNDER;

            Position() = default;
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

            /// @brief Convert the enum to a string
            operator std::string() const {
                switch (value) {
                    case Value::ALL_ROUNDER: return "ALL_ROUNDER";
                    case Value::STRIKER: return "STRIKER";
                    case Value::DEFENDER: return "DEFENDER";
                    case Value::GOALIE: return "GOALIE";
                    case Value::DYNAMIC: return "DYNAMIC";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            /// @brief Get the full yaml name of the config for the current position
            /// @return The full yaml name of the config for the current position
            std::string get_config_name() {
                switch (value) {
                    case Value::ALL_ROUNDER: return "AllRounder.yaml";
                    case Value::STRIKER: return "Striker.yaml";
                    case Value::DEFENDER: return "Defender.yaml";
                    case Value::GOALIE: return "Goalie.yaml";
                    case Value::DYNAMIC: return "Dynamic.yaml";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }
            }

            /// @brief Increment the enum, for toggle
            void operator++() {
                switch (value) {
                    case Value::ALL_ROUNDER: value = Value::STRIKER; break;
                    case Value::STRIKER: value = Value::GOALIE; break;
                    case Value::GOALIE: value = Value::DEFENDER; break;
                    case Value::DEFENDER: value = Value::DYNAMIC; break;
                    case Value::DYNAMIC: value = Value::ALL_ROUNDER; break;
                    default: value = Value::STRIKER;
                }
            }
        } robot_position;


        /// @brief Display values
        struct Display {
            /// @brief Enum for options in first column
            enum class Column1 { ROBOT_NAME, WIFI_INTERFACE, IP_ADDRESS, SSID, PASSWORD, END };
            /// @brief Enum for options in second column
            enum class Column2 { PLAYER_ID, TEAM_ID, POSITION, END };
            /// @brief Column 1 padding
            static const size_t C1_PAD = 2;
            /// @brief Column 1 selection position
            static const size_t C1_SEL_POS = 20;
            /// @brief Column 2 padding
            static const size_t C2_PAD = 40;
            /// @brief Column 2 selection position
            static const size_t C2_SEL_POS = 51;
            /// @brief Selection highlight width
            static const size_t SELECT_WIDTH = 18;
            /// @brief How much to pad the commands from the bottom of the window
            static const size_t COMMAND_BOTTOM_PAD = 10;
            /// @brief How much to pad the log message from the bottom of the window
            static const size_t LOG_BOTTOM_PAD = 2;
            /// @brief Gap between commands
            static const size_t COMMAND_GAP = 17;
            /// @brief The index of the item the user is selecting
            size_t row_selection = 0;
            /// @brief Index of the column the user is selecting
            size_t column_selection = 0;
            /// @brief The log message to print to the user at the bottom of the window
            std::string log_message = "";
        } display{};

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
        /// The order is: robot-name-specific, hostname-specific, platform-specific, and finally the default config.
        /// @param filename The config file to find
        /// @return The correct config file path according to config rules
        std::string get_config_file(std::string filename);


    public:
        /// @brief Called by the powerplant to build and setup the RoboCupConfiguration reactor.
        explicit RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
