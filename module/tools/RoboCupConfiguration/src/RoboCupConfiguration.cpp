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

#include <sstream>

#include "extension/Configuration.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include "utility/support/network.hpp"

namespace module::tools {

    using extension::Configuration;

    RoboCupConfiguration::RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RoboCupConfiguration.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RoboCupConfiguration.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            hostname        = utility::support::getHostname();
        });

        on<Startup>().then([this] {
            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users messages
            noecho();
            // Hide the cursor
            curs_set(0);

            set_values();
            refresh_view();
        });
        // When we shutdown end ncurses
        on<Shutdown>().then(endwin);

        // Trigger when stdin has something to read
        on<IO>(STDIN_FILENO, IO::READ).then([this] {
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
                        // if it didn't work, don't do anything
                    }
                    break;
                case ' ':  // Toggles selection
                    toggle_selection();
                    break;
                case 'R':  // updates visual changes
                    refresh_view();
                    break;
                case 'C':  // configures files with the new values
                    configure();
                    break;
                case 'X':  // shutdowns powerplant
                    powerplant.shutdown();
                    break;
            }

            // Update whatever visual changes we made
            refresh_view();
        });
    }

    void RoboCupConfiguration::set_values() {
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
    }

    void RoboCupConfiguration::configure() {
        // For testing in docker lol
        hostname = "nugus4";

        // Configure the game files
        std::string soccer_file = get_config_file("Soccer.yaml");

        // Write to the yaml file
        YAML::Node config  = YAML::LoadFile(soccer_file);
        config["position"] = std::string(robot_position);
        std::ofstream file(soccer_file);
        file << config;
        file.close();

        // Configure the network files
        // Check if we are on a robot
        if (get_platform() != "nugus") {
            log<NUClear::WARN>("Network configuration only available on NUgus robots.");
            return;
        }

        // Get folder name
        std::string folder = fmt::format("system/{}/etc/systemd/network", hostname);

        // File 40-wifi-robocup.network rename to 30-wifi.network so it is used instead of the default
        std::string old_file = fmt::format("{}/40-wifi-robocup.network", folder);
        std::string new_file = fmt::format("{}/30-wifi.network", folder);
        std::rename(old_file.c_str(), new_file.c_str());

        // Change third component of ip_address with team_id and fourth component with player_id
        std::stringstream ss(ip_address);
        std::vector<std::string> ip_parts{};
        std::string part;
        while (std::getline(ss, part, '.')) {
            ip_parts.push_back(part);
        }
        ip_address          = fmt::format("{}.{}.{}.{}", ip_parts[0], ip_parts[1], team_id, player_id);
        std::string gateway = fmt::format("{}.{}.3.1", ip_parts[0], ip_parts[1]);

        // Write the new ip address to the file
        std::ofstream n_file(new_file);
        n_file << "[Match]\nName=wlp0s20f3\n\n[Network]\nAddress=" << ip_address << "/16\nGateway=" << gateway
               << "\nDNS=" << gateway << "\nDNS=8.8.8.8";
        n_file.close();

        // Configure the wpa_supplicant file
        std::string wpa_supplicant_file = "system/default/etc/wpa_supplicant/wpa_supplicant-wlp0s20f3.conf";
        std::ofstream wpa_file(wpa_supplicant_file);
        wpa_file
            << "ctrl_interface=/var/run/wpa_supplicant\nctrl_interface_group=wheel\nupdate_config=1\nfast_reauth=1 "
            << "\nap_scan = 1\n\nnetwork = "
            << " { "
            << "\n\tssid =\"" << ssid << "\"\n\tpsk=\"" << password << "\"\n\tpriority=1\n}";
        wpa_file.close();
    }
    void RoboCupConfiguration::toggle_selection() {
        // Networking configuration column
        if (column_selection == 0) {
            switch (row_selection) {
                case 2:  // player_id
                    player_id = player_id == 5 ? 1 : player_id + 1;
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
        if (std::filesystem::exists(fmt::format("config/{}/{}", hostname, filename))) {
            return fmt::format("config/{}/{}", hostname, filename);
        }
        if (std::filesystem::exists(fmt::format("config/{}/{}", get_platform(), filename))) {
            return fmt::format("config/{}/{}", get_platform(), filename);
        }
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
        mvprintw(4, 2, ("Hostname  : " + hostname).c_str());
        mvprintw(5, 2, ("Wifi Interface  : " + wifi_interface).c_str());
        mvprintw(6, 2, ("IP Address: " + ip_address).c_str());
        mvprintw(7, 2, ("Player ID : " + std::to_string(player_id)).c_str());
        mvprintw(8, 2, ("Team ID   : " + std::to_string(team_id)).c_str());
        mvprintw(9, 2, ("SSID      : " + ssid).c_str());
        mvprintw(10, 2, ("Password  : " + password).c_str());

        attron(A_ITALIC);
        mvprintw(3, 30, "Game Configuration");
        attroff(A_ITALIC);
        mvprintw(4, 30, ("Position: " + std::string(robot_position)).c_str());

        std::stringstream ready_string{};
        ready_string << ready_position.transpose();
        mvprintw(4, 30, ("Ready   : " + ready_string.str()).c_str());

        // Highlight our selected point
        mvchgat(row_selection + 4, 14 + (column_selection * 26), 8, A_STANDOUT, 0, nullptr);


        refresh();
    }


}  // namespace module::tools
