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
#include "RoboCupConfiguration.hpp"

#include <algorithm>
#include <ranges>
#include <sstream>

#include "extension/Configuration.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include "utility/support/network.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    RoboCupConfiguration::RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RoboCupConfiguration.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RoboCupConfiguration.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users log_messages
            noecho();
            // Hide the cursor
            curs_set(0);

            // Set the fields and show them
            get_config_values();
            refresh_view();
        });

        // When we shutdown end ncurses
        on<Shutdown>().then(endwin);

        // Trigger when stdin has something to read
        on<IO>(STDIN_FILENO, IO::READ).then([this] {
            log_message = "";

            // Get the character the user has typed
            switch (getch()) {
                case KEY_UP:  // Change row_selection up
                    row_selection = row_selection != 0 ? row_selection - 1 : (column_selection ? 1 : 5);
                    break;
                case KEY_DOWN:  // Change row_selection down
                    row_selection = column_selection ? (row_selection + 1) % 2 : (row_selection + 1) % 6;
                    break;
                case KEY_LEFT:  // Network config
                    column_selection = 0;
                    break;
                case KEY_RIGHT:  // Game config
                    column_selection = 1;
                    row_selection    = row_selection > 1 ? 1 : row_selection;
                    break;
                case '\n':       // Edit selected field
                case KEY_ENTER:  // Edit selected field
                    try {
                        edit_selection();
                    }
                    catch (...) {
                        log_message = "Warning: Invalid input!";
                    }
                    break;
                case ' ':  // Toggles selection
                    toggle_selection();
                    break;
                case 'R':  // reset the values
                    get_config_values();
                    log_message = "Values have been reset.";
                    break;
                case 'C':  // configures files with the new values
                    set_config_values();
                    break;
                case 'N':  // Apply any changes to the network settings
                    configure_network();
                    break;
                case 'S':  // starts the robocup service
                    system("systemctl start robocup");
                    log_message = "RoboCup service started!";
                    break;
                case 'D':  // stops the robocup service
                    system("systemctl stop robocup");
                    log_message = "RoboCup service stopped!";
                    break;
                case 'E':  // enables wifi services
                    system("systemctl enable --now wpa_supplicant");
                    system("systemctl enable --now systemd-networkd");
                    break;
                case 'W':  // disables wifi services
                    system("systemctl disable --now wpa_supplicant");
                    system("systemctl disable --now systemd-networkd");
                    break;
                case 'X':  // shutdown powerplant
                    powerplant.shutdown();
                    break;
            }

            // Update whatever visual changes we made
            refresh_view();
        });
    }

    void RoboCupConfiguration::get_config_values() {
        // Hostname
        hostname = utility::support::get_hostname();

        // Wifi interface
        wifi_interface = utility::support::get_wireless_interface();

        // IP Address
        ip_address = utility::support::get_ip_address(wifi_interface);

        // Team ID
        YAML::Node global_config = YAML::LoadFile(get_config_file("GlobalConfig.yaml"));
        team_id                  = global_config["team_id"].as<int>();
        player_id                = global_config["player_id"].as<int>();

        // SSID
        ssid = utility::support::get_ssid(wifi_interface);

        // Password
        password = utility::support::get_wifi_password(ssid, wifi_interface);

        // Robot position
        {
            std::string soccer_file = get_config_file("Soccer.yaml");
            YAML::Node config       = YAML::LoadFile(soccer_file);
            robot_position          = config["position"].as<std::string>();
        }
        // Ready position
        {
            std::string ready_file = get_config_file(robot_position.get_config_name());
            YAML::Node config      = YAML::LoadFile(ready_file);
            ready_position         = config["ready_position"].as<Eigen::Vector3d>();
        }

        // Check if we have permissions
        if (geteuid() != 0) {
            log_message = "Warning: To configure the network, run with sudo.";
        }
    }

    void RoboCupConfiguration::configure_network() {
        // In case values aren't set yet, set them
        set_config_values();

        // Check if we are on a robot
        if (get_platform() != "nugus") {
            log_message = "Configure Error: Network configuration only available on NUgus robots!";
            return;
        }

        // Check if we have permissions
        if (geteuid() != 0) {
            log_message = "Configure Error: Insufficient permissions! Run with sudo!";
            return;
        }

        // Copy the network files
        std::filesystem::copy_file(fmt::format("system/{}/etc/systemd/network/30-wifi.network", hostname),
                                   "/etc/systemd/network/30-wifi.network",
                                   std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(
            fmt::format("system/{}/etc/wpa_supplicant/wpa_supplicant-{}.conf", hostname, wifi_interface),
            fmt::format("/etc/wpa_supplicant/wpa_supplicant-{}.conf", wifi_interface),
            std::filesystem::copy_options::overwrite_existing);

        // Restart the network
        system("systemctl restart systemd-networkd");
        system("systemctl restart wpa_supplicant");
        system(("systemctl restart wpa_supplicant@" + wifi_interface).c_str());

        log_message = "Network configured!";
    }

    void RoboCupConfiguration::set_config_values() {
        /* GAME CONFIG */
        {  // Write the robot's position to the soccer file
            std::string soccer_file = get_config_file("Soccer.yaml");
            // Write to the yaml file
            YAML::Node config  = YAML::LoadFile(soccer_file);
            config["position"] = std::string(robot_position);
            std::ofstream file(soccer_file);
            file << config;
        }

        {
            // Write ready position to the corresponding config file
            std::string ready_file   = get_config_file(robot_position.get_config_name());
            YAML::Node config        = YAML::LoadFile(ready_file);
            config["ready_position"] = YAML::Node(ready_position);
            std::ofstream file(ready_file);
            file << config;
        }

        // Check if we are on a robot
        if (get_platform() != "nugus") {
            log_message = "Configure Error: Network configuration only available on NUgus robots!";
            return;
        }

        // Check if any of ip_address, ssid or password are empty
        if (ip_address.empty() || ssid.empty() || password.empty()) {
            log_message = "Configure Error: IP Address, SSID and Password must be set!";
            return;
        }

        /* WIRELESS NETWORK SYSTEMD CONFIG */

        // Get folder name
        const std::string systemd_folder = fmt::format("system/{}/etc/systemd/network", hostname);

        // Parse the IP address
        std::stringstream ss(ip_address);
        std::vector<std::string> ip_parts{};
        for (std::string part; std::getline(ss, part, '.');) {
            ip_parts.push_back(part);
        }

        // Check if team_id and player_id match with third and fourth parts of the IP address.
        // This shouldn't break things, but the user should know that there may be an issue.
        // In the lab, the IP addresses of the robots are set and may not match in this way,
        // but at RoboCup, it is important that this format is followed
        if (team_id != std::stoi(ip_parts[2]) || player_id != std::stoi(ip_parts[3])) {
            log_message +=
                "Warning: At RoboCup, the third position of the IP address should be the team ID and the fourth should "
                "be the player number. ";
        }

        // Write the new ip address to the file
        std::ofstream(fmt::format("{}/30-wifi.network", systemd_folder))
            << fmt::format("[Match]\nName={}\n\n[Network]\nAddress={}/16\nGateway={}.{}.3.1\nDNS=8.8.8.8",
                           wifi_interface,
                           ip_address,
                           ip_parts[0],
                           ip_parts[1]);


        /* WPA_SUPPLICANT CONFIG */

        // Make a new wpa_supplicant file in the robot-specific directory
        // This is so that we don't lose the original if we need it
        // And so when we append to it with a high priority, we are doing it from fresh
        // And will not encounter any issues when running multiple times
        std::string wpa_supplicant_dir  = fmt::format("system/{}/etc/wpa_supplicant", hostname);
        std::string wpa_supplicant_file = fmt::format("{}/wpa_supplicant-{}.conf", wpa_supplicant_dir, wifi_interface);
        system(("mkdir -p " + wpa_supplicant_dir).c_str());

        std::filesystem::copy_file(
            fmt::format("system/default/etc/wpa_supplicant/wpa_supplicant-{}.conf", wifi_interface),
            wpa_supplicant_file,
            std::filesystem::copy_options::overwrite_existing);

        // Append to the wpa_supplicant file with a high priority
        std::string command =
            "wpa_passphrase " + ssid + " " + password + " | sed 's/}/\tpriority=9999\\n}/' >> " + wpa_supplicant_file;
        system(command.c_str());

        log_message += "Files have been configured.";
    }

    void RoboCupConfiguration::toggle_selection() {
        // Networking configuration column
        if (column_selection == 0) {
            switch (row_selection) {
                case 2:  // player_id
                    player_id = player_id == 6 ? 1 : player_id + 1;
                    break;
                case 3:  // team_id
                    team_id = team_id == 33 ? 1 : team_id + 1;
                    break;
            }
            return;
        }
        // Game configuration column
        if (row_selection == 0) {
            ++robot_position;
            {
                // Get ready position for this position
                std::string ready_file = get_config_file(robot_position.get_config_name());
                YAML::Node config      = YAML::LoadFile(ready_file);
                ready_position         = config["ready_position"].as<Eigen::Vector3d>();
            }
        }
    }

    void RoboCupConfiguration::edit_selection() {
        // Networking configuration column
        if (column_selection == 0) {
            switch (row_selection) {
                case 0:  // wifi interface
                    wifi_interface = user_input();
                    break;
                case 1:  // ip_address
                    ip_address = user_input();
                    break;
                case 2:  // player_id
                    player_id = std::stoi(user_input());
                    break;
                case 3:  // team_id
                    team_id = std::stoi(user_input());
                    break;
                case 4:  // ssid
                    ssid = user_input();
                    break;
                case 5:  // password
                    password = user_input();
                    break;
            }
            return;
        }
        // Game configuration column
        switch (row_selection) {
            case 0:  // robot_position
                robot_position = user_input();
                {
                    // Get ready position for this position
                    std::string ready_file = get_config_file(robot_position.get_config_name());
                    YAML::Node config      = YAML::LoadFile(ready_file);
                    ready_position         = config["ready_position"].as<Eigen::Vector3d>();
                }
                break;
            case 1:  // ready position
                std::stringstream ss(user_input());
                ss >> ready_position.x() >> ready_position.y() >> ready_position.z();
                break;
        }
    }

    std::string RoboCupConfiguration::user_input() {
        // Read characters until we see either esc or enter
        std::stringstream chars;

        // Keep reading until our termination case is reached
        while (true) {
            auto ch = getch();
            switch (ch) {
                case 27: return "";
                case '\n':
                case KEY_ENTER: return chars.str(); break;
                default:
                    chars << static_cast<char>(ch);
                    addch(ch);
                    break;
            }
        }
    }

    std::string RoboCupConfiguration::get_platform() {
        // It is assumed that all hostnames are in the format <platform name><robot number>,
        // such that the regular expression
        // [a-z]+[0-9]+?
        // will match all hostnames
        std::regex re("([a-z]+)([0-9]+)?");
        std::smatch match;

        if (std::regex_match(hostname, match, re)) {
            // match[0] will be the full string
            // match[1] the first match (platform name)
            // match[2] the second match (robot number)
            return match[1].str();
        }

        // If platform cannot be found, return empty
        return "";
    }

    std::string RoboCupConfiguration::get_config_file(std::string filename) {
        // First check hostname-specific
        if (std::filesystem::exists(fmt::format("config/{}/{}", hostname, filename))) {
            return fmt::format("config/{}/{}", hostname, filename);
        }
        // Then check platform-specific
        if (std::filesystem::exists(fmt::format("config/{}/{}", get_platform(), filename))) {
            return fmt::format("config/{}/{}", get_platform(), filename);
        }
        // Finally, check the default
        return "config/" + filename;
    }

    void RoboCupConfiguration::refresh_view() {
        // Clear our window
        erase();

        // Outer box
        box(stdscr, 0, 0);

        // Write our title
        attron(A_BOLD);
        mvprintw(0, (COLS - 14) / 2, " RoboCup Configuration ");
        attroff(A_BOLD);

        attron(A_ITALIC);
        mvprintw(2, 2, "Networking");
        attroff(A_ITALIC);
        mvprintw(4, 2, ("Hostname        : " + hostname).c_str());
        mvprintw(5, 2, ("Wifi Interface  : " + wifi_interface).c_str());
        mvprintw(6, 2, ("IP Address      : " + ip_address).c_str());
        mvprintw(7, 2, ("Player ID       : " + std::to_string(player_id)).c_str());
        mvprintw(8, 2, ("Team ID         : " + std::to_string(team_id)).c_str());
        mvprintw(9, 2, ("SSID            : " + ssid).c_str());
        mvprintw(10, 2, ("Password        : " + password).c_str());

        attron(A_ITALIC);
        mvprintw(2, 40, "Game Configuration");
        attroff(A_ITALIC);
        mvprintw(5, 40, ("Position: " + std::string(robot_position)).c_str());

        std::stringstream ready_string{};
        ready_string << ready_position.transpose();
        mvprintw(6, 40, ("Ready   : " + ready_string.str()).c_str());

        // Print commands
        // Heading Commands
        attron(A_BOLD);
        mvprintw(LINES - 10, 2, "Commands");
        attroff(A_BOLD);

        // Each Command
        const char* COMMANDS[] = {"ENTER", "SPACE", "F", "R", "C", "N", "S", "D", "E", "W", "X"};

        // Each Meaning
        const char* MEANINGS[] = {"Edit",
                                  "Toggle",
                                  "Refresh",
                                  "Reset",
                                  "Configure",
                                  "Network",
                                  "Start RoboCup",
                                  "Stop RoboCup",
                                  "Enable Wifi",
                                  "Disable Wifi",
                                  "Shutdown"};

        // Prints commands and their meanings to the screen
        for (size_t i = 0; i < (sizeof(COMMANDS) / sizeof(COMMANDS[0])); i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - 9, 2 + ((3 + 14) * (i / 2)), COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            int gap = i == 0 ? 8 : 4;
            mvprintw(LINES - 9, gap + ((3 + 14) * (i / 2)), MEANINGS[i]);
        }

        for (size_t i = 1; i < (sizeof(COMMANDS) / sizeof(COMMANDS[0])); i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - 8, 2 + ((3 + 14) * ((i - 1) / 2)), COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            int gap = i == 1 ? 8 : 4;
            mvprintw(LINES - 8, gap + ((3 + 14) * ((i - 1) / 2)), MEANINGS[i]);
        }

        // Print any log_messages
        attron(A_BOLD);
        mvprintw(LINES - 2, 2, log_message.c_str());
        attroff(A_BOLD);

        // Highlight our selected point
        mvchgat(row_selection + 5, 20 + (column_selection * 30), 13, A_STANDOUT, 0, nullptr);

        refresh();
    }

}  // namespace module::tools
