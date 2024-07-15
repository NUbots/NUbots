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
#include <termios.h>

#include "extension/Configuration.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include "utility/platform/aliases.hpp"
#include "utility/support/network.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    RoboCupConfiguration::RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("RoboCupConfiguration.yaml").then([this](const Configuration& config) {
            // Use configuration here from file RoboCupConfiguration.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.wifi_networks = config["wifi_networks"].as<std::map<std::string, std::string>>();
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
            display.log_message = "";

            // Get the character the user has typed
            switch (getch()) {
                case KEY_UP:  // Change row_selection up
                    display.row_selection = display.row_selection != 0
                                                ? display.row_selection - 1
                                                : (display.column_selection ? size_t(Display::Column2::END) - 1
                                                                            : size_t(Display::Column1::END) - 1);
                    break;
                case KEY_DOWN:  // Change row_selection down
                    display.row_selection = display.column_selection
                                                ? (display.row_selection + 1) % size_t(Display::Column2::END)
                                                : (display.row_selection + 1) % size_t(Display::Column1::END);
                    break;
                case KEY_LEFT:  // Network config
                    display.column_selection = 0;
                    break;
                case KEY_RIGHT:  // Game config
                    display.column_selection = 1;
                    display.row_selection    = display.row_selection > size_t(Display::Column2::END) - 1
                                                   ? size_t(Display::Column2::END) - 1
                                                   : display.row_selection;
                    break;
                case '\n':       // Edit selected field
                case KEY_ENTER:  // Edit selected field
                    try {
                        edit_selection();
                    }
                    catch (...) {
                        display.log_message = "Warning: Invalid input!";
                    }
                    break;
                case ' ':  // Toggles selection
                    toggle_selection();
                    break;
                case 'R':  // reset the values
                    get_config_values();
                    display.log_message = "Values have been reset.";
                    break;
                case 'C':  // configures files with the new values
                    set_config_values();
                    break;
                case 'N':  // Apply any changes to the network settings
                    configure_network();
                    break;
                case 'S':  // starts the robocup service
                    system("systemctl start robocup");
                    display.log_message = "RoboCup service started!";
                    break;
                case 'D':  // stops the robocup service
                    system("systemctl stop robocup");
                    display.log_message = "RoboCup service stopped!";
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

        // Robot name
        robot_name = utility::platform::get_robot_alias(hostname);

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

        // Check if we have permissions
        if (geteuid() != 0) {
            display.log_message = "Warning: To configure the network, run with sudo.";
        }
    }

    void RoboCupConfiguration::configure_network() {
        // In case values aren't set yet, set them
        set_config_values();

        // Check if we are on a robot
        if (get_platform() != "nugus") {
            display.log_message = "Configure Error: Network configuration only available on NUgus robots!";
            return;
        }

        // Check if we have permissions
        if (geteuid() != 0) {
            display.log_message = "Configure Error: Insufficient permissions! Run with sudo!";
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

        display.log_message = "Network configured!";
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
            // Write team_id and player_id to the global config
            std::string global_file = get_config_file("GlobalConfig.yaml");
            YAML::Node config       = YAML::LoadFile(global_file);
            config["team_id"]       = team_id;
            config["player_id"]     = player_id;
            std::ofstream file(global_file);
            file << config;
        }

        // Check if we are on a robot
        if (get_platform() != "nugus") {
            display.log_message = "Configure Error: Network configuration only available on NUgus robots!";
            return;
        }

        // Write robot name
        if (!robot_name.empty()) {
            utility::platform::set_robot_alias(hostname, robot_name);
        }

        // Check if any of ip_address, ssid or password are empty
        if (ip_address.empty() || ssid.empty() || password.empty()) {
            display.log_message = "Configure Error: IP Address, SSID and Password must be set!";
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
            display.log_message +=
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

        display.log_message += "Files have been configured.";
    }

    void RoboCupConfiguration::toggle_selection() {
        // Networking configuration column
        if (display.column_selection == 0) {
            if (display.row_selection == int(Display::Column1::SSID) && !cfg.wifi_networks.empty()) {
                // See if the current SSID is in the cfg.wifi_networks map
                auto it = cfg.wifi_networks.find(ssid);
                if (it != cfg.wifi_networks.end()) {
                    // If it is, set the SSID and password to the next value in the map
                    auto next = std::next(it);
                    ssid      = next == cfg.wifi_networks.end() ? cfg.wifi_networks.begin()->first : next->first;
                    password  = next == cfg.wifi_networks.end() ? cfg.wifi_networks.begin()->second : next->second;
                }
                else {
                    // If it isn't, set the SSID and password to the first value in the map
                    ssid     = cfg.wifi_networks.begin()->first;
                    password = cfg.wifi_networks.begin()->second;
                }
            }
            return;
        }
        // Game configuration column
        Display::Column2 column = static_cast<Display::Column2>(display.row_selection);
        switch (column) {
            case Display::Column2::PLAYER_ID: player_id = player_id == MAX_PLAYER_ID ? 1 : player_id + 1; break;
            case Display::Column2::TEAM_ID: team_id = team_id == MAX_TEAM_ID ? 1 : team_id + 1; break;
            case Display::Column2::POSITION: ++robot_position; break;
            default: break;
        }
    }

    void RoboCupConfiguration::edit_selection() {
        // Networking configuration column
        if (display.column_selection == 0) {
            Display::Column1 column = static_cast<Display::Column1>(display.row_selection);
            switch (column) {
                case Display::Column1::ROBOT_NAME: robot_name = user_input(); break;
                case Display::Column1::WIFI_INTERFACE: wifi_interface = user_input(); break;
                case Display::Column1::IP_ADDRESS: ip_address = user_input(); break;
                case Display::Column1::SSID: ssid = user_input(); break;
                case Display::Column1::PASSWORD: password = user_input(); break;
                default: break;
            }
            return;
        }
        // Game configuration column
        Display::Column2 column = static_cast<Display::Column2>(display.row_selection);
        switch (column) {
            case Display::Column2::PLAYER_ID: player_id = std::stoi(user_input()); break;
            case Display::Column2::TEAM_ID: team_id = std::stoi(user_input()); break;
            case Display::Column2::POSITION: robot_position = user_input(); break;
            default: break;
        }
    }

    std::string RoboCupConfiguration::user_input() {
        // Read characters until we see either esc or enter
        std::string input;
        int cursor_pos = 0;

        // Keep reading until our termination case is reached
        while (true) {
            auto ch = getch();
            switch (ch) {
                case 27: return "";  // Escape key
                case '\n':
                case KEY_ENTER: return input;
                case KEY_BACKSPACE:
                case 127:  // ASCII code for backspace
                    if (!input.empty() && cursor_pos > 0) {
                        input.erase(cursor_pos - 1, 1);
                        cursor_pos--;
                        // Redraw the input
                        move(getcury(stdscr), display.C1_SEL_POS);
                        clrtoeol();
                        addstr(input.c_str());
                        move(getcury(stdscr), display.C1_SEL_POS + cursor_pos);
                    }
                    break;
                default:
                    if (isprint(ch)) {  // Only add printable characters
                        input.insert(cursor_pos, 1, static_cast<char>(ch));
                        cursor_pos++;
                        addch(ch);
                    }
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
        // First check name-specific config
        std::string robot_name = utility::platform::get_robot_alias(hostname);
        if (!robot_name.empty() && std::filesystem::exists(fmt::format("config/{}/{}", robot_name, filename))) {
            return fmt::format("config/{}/{}", robot_name, filename);
        }
        // Next check hostname-specific
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
        mvprintw(2, display.C1_PAD, "Networking");
        attroff(A_ITALIC);
        mvprintw(4, display.C1_PAD, ("Hostname        : " + hostname).c_str());
        mvprintw(5, display.C1_PAD, ("Robot Name      : " + robot_name).c_str());
        mvprintw(6, display.C1_PAD, ("Wifi Interface  : " + wifi_interface).c_str());
        mvprintw(7, display.C1_PAD, ("IP Address      : " + ip_address).c_str());
        mvprintw(8, display.C1_PAD, ("SSID            : " + ssid).c_str());
        mvprintw(9, display.C1_PAD, ("Password        : " + password).c_str());

        attron(A_ITALIC);
        mvprintw(2, display.C2_PAD, "Game Configuration");
        attroff(A_ITALIC);
        mvprintw(5, display.C2_PAD, ("Player ID: " + std::to_string(player_id)).c_str());
        mvprintw(6, display.C2_PAD, ("Team ID  : " + std::to_string(team_id)).c_str());
        mvprintw(7, display.C2_PAD, ("Position : " + std::string(robot_position)).c_str());

        // Print commands
        // Heading Commands
        attron(A_BOLD);
        mvprintw(LINES - display.COMMAND_BOTTOM_PAD, display.C1_PAD, "Commands");
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
        // Print the first row
        for (size_t i = 0; i < (sizeof(COMMANDS) / sizeof(COMMANDS[0])); i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - display.COMMAND_BOTTOM_PAD + 1,
                     display.C1_PAD + (display.COMMAND_GAP * (i / 2)),
                     COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            int gap = i == 0 ? 8 : 4;  // ENTER and SPACE are longer than the single-letter commands
            mvprintw(LINES - display.COMMAND_BOTTOM_PAD + 1, gap + (display.COMMAND_GAP * (i / 2)), MEANINGS[i]);
        }

        // Print the second row
        for (size_t i = 1; i < (sizeof(COMMANDS) / sizeof(COMMANDS[0])); i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - display.COMMAND_BOTTOM_PAD + 2,
                     display.C1_PAD + (display.COMMAND_GAP * ((i - 1) / 2)),
                     COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            int gap = i == 1 ? 8 : 4;  // ENTER and SPACE are longer than the single-letter commands
            mvprintw(LINES - display.COMMAND_BOTTOM_PAD + 2, gap + (display.COMMAND_GAP * ((i - 1) / 2)), MEANINGS[i]);
        }

        // Print any log_messages
        attron(A_BOLD);
        mvprintw(LINES - display.LOG_BOTTOM_PAD, display.C1_PAD, display.log_message.c_str());
        attroff(A_BOLD);

        // Highlight our selected point
        int col_pos = display.column_selection == 0 ? display.C1_SEL_POS : display.C2_SEL_POS;
        mvchgat(display.row_selection + 5, col_pos, display.SELECT_WIDTH, A_STANDOUT, 0, nullptr);

        refresh();
    }

}  // namespace module::tools
