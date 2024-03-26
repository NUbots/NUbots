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

#include "extension/Configuration.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include "utility/support/hostname.hpp"


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

            refresh_view();
        });
        // When we shutdown end ncurses
        on<Shutdown>().then(endwin);

        // Trigger when stdin has something to read
        on<IO>(STDIN_FILENO, IO::READ).then([this] {
            // Get the character the user has typed
            switch (getch()) {
                case KEY_UP:  // Change row_selection up
                    row_selection = column_selection ? (row_selection - 1) % 2 : (row_selection - 1) % 5;
                    break;
                case KEY_DOWN:  // Change row_selection down
                    row_selection = column_selection ? (row_selection + 1) % 2 : (row_selection + 1) % 5;
                    break;
                case KEY_LEFT:  // Network config
                    column_selection = 0;
                    break;
                case KEY_RIGHT:  // Game config
                    column_selection = 1;
                    row_selection    = row_selection > 1 ? 1 : row_selection;
                    break;
                // case ',':  // Move left a frame
                //     activate_frame(frame == 0 ? frame : frame - 1);
                //     break;
                // case '.':  // Move right a frame
                //     activate_frame(frame == script.frames.size() - 1 ? frame : frame + 1);
                //     break;
                // case '\n':       // Edit selected field
                // case KEY_ENTER:  // Edit selected field
                //     edit_row_selection();
                //     break;
                // case ' ':  // Toggle lock mode
                //     toggle_lock_motor();
                //     break;
                // case 'R':  // updates visual changes
                //     refresh_view();
                //     break;
                // // case 'M': mirror_script(); break;
                // case 'G':  // allows multiple gains to be edited at once
                //     // edit_gain();
                //     break;
                // case ':':  // lists commands
                //     // help();
                //     break;
                case 'X':  // shutdowns powerplant
                    powerplant.shutdown();
                    break;
            }

            // Update whatever visual changes we made
            refresh_view();
        });
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
        mvprintw(3, 2, ("Hostname  : " + hostname).c_str());
        mvprintw(5, 2, ("IP Address: " + ip_address).c_str());
        mvprintw(4, 2, ("Player ID : " + std::to_string(player_id)).c_str());
        mvprintw(5, 2, ("Team ID   : " + std::to_string(team_id)).c_str());
        mvprintw(6, 2, ("SSID      : " + hostname).c_str());
        mvprintw(7, 2, ("Password  : " + hostname).c_str());

        attron(A_ITALIC);
        mvprintw(2, 30, "Game Configuration");
        attroff(A_ITALIC);
        mvprintw(3, 30, ("Position: " + std::string(robot_position)).c_str());
        mvprintw(4, 30, "Ready   : [1, 0, 0]");

        // Highlight our selected point
        mvchgat(row_selection + 3, 14 + (column_selection * 26), 6, A_STANDOUT, 0, nullptr);


        refresh();
    }


}  // namespace module::tools
