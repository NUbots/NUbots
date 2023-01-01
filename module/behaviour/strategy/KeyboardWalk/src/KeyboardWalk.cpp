/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "KeyboardWalk.hpp"

#include <clocale>
#include <csignal>
#include <cstdio>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/HeadCommand.hpp"
#include "message/motion/KickCommand.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::strategy {

    using message::behaviour::MotionCommand;
    using message::motion::HeadCommand;
    using NUClear::message::LogMessage;
    using LimbID = utility::input::LimbID;

    void quit() {
        endwin();
        std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
    }

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), velocity(Eigen::Vector2f::Zero()) {

        // Ensure UTF-8 is enabled
        std::setlocale(LC_ALL, "en_US.UTF-8");

        // Start curses mode
        initscr();

        // Initialise colours
        if (has_colors()) {
            colours_enabled = true;
            start_color();
            init_pair(short(LogColours::TRACE_COLOURS), COLOR_WHITE, COLOR_BLACK);
            init_pair(short(LogColours::DEBUG_COLOURS), COLOR_GREEN, COLOR_BLACK);
            init_pair(short(LogColours::INFO_COLOURS), COLOR_CYAN, COLOR_BLACK);
            init_pair(short(LogColours::WARN_COLOURS), COLOR_YELLOW, COLOR_BLACK);
            init_pair(short(LogColours::ERROR_COLOURS), COLOR_MAGENTA, COLOR_BLACK);
            init_pair(short(LogColours::FATAL_COLOURS), COLOR_RED, COLOR_BLACK);
        }
        else {
            colours_enabled = false;
        }

        // Capture our characters immediately (but pass through signals)
        cbreak();
        // Capture arrows and function keys
        keypad(stdscr, true);
        // Don't echo the users messages
        noecho();

        // Set up windows
        create_windows();

        // Trigger when stdin has something to read
        on<IO>(STDIN_FILENO, IO::READ).then([this] {
            switch (tolower(getch())) {
                case 'w': forward(); break;
                case 'a': left(); break;
                case 's': back(); break;
                case 'd': right(); break;
                case 'z': turn_left(); break;
                case 'x': turn_right(); break;
                case 'r': reset(); break;
                case 'g': get_up(); break;
                case 'e': walk_toggle(); break;
                case '.': kick(LimbID::RIGHT_LEG); break;
                case ',': kick(LimbID::LEFT_LEG); break;
                case KEY_LEFT: look_left(); break;
                case KEY_RIGHT: look_right(); break;
                case KEY_UP: look_up(); break;
                case KEY_DOWN: look_down(); break;
                case 'q': quit(); return;
                default:
                    log<NUClear::ERROR>("Unknown Command");
                    print_status();
                    break;
            }
        });

        on<Trigger<LogMessage>>().then([this](const LogMessage& packet) {
            // Where this message came from
            std::string source = "";

            // If we know where this log message came from, we display that
            if (packet.task != nullptr) {
                // Get our reactor name
                std::string reactor = packet.task->identifier[1];

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                // This is our source
                source =
                    reactor + " " + (packet.task->identifier[0].empty() ? "" : "- " + packet.task->identifier[0] + " ");
            }

            LogColours colours;
            switch (packet.level) {
                default:
                case NUClear::TRACE: colours = LogColours::TRACE_COLOURS; break;
                case NUClear::DEBUG: colours = LogColours::DEBUG_COLOURS; break;
                case NUClear::INFO: colours = LogColours::INFO_COLOURS; break;
                case NUClear::WARN: colours = LogColours::WARN_COLOURS; break;
                case NUClear::ERROR: colours = LogColours::ERROR_COLOURS; break;
                case NUClear::FATAL: colours = LogColours::FATAL_COLOURS; break;
            }

            update_window(log_window, colours, source, packet.message, true);
        });

        on<Shutdown>().then(endwin);

        update_command();
        print_status();
    }

    void KeyboardWalk::create_windows() {
        command_window = std::shared_ptr<WINDOW>(newwin(5, COLS, 0, 0), [](auto p) {
            /* box(local_win, ' ', ' '); : This won't produce the desired
             * result of erasing the window. It will leave it's four corners
             * and so an ugly remnant of window.
             */
            wborder(p, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
            /* The parameters taken are
             * 1. win: the window on which to operate
             * 2. ls: character to be used for the left side of the window
             * 3. rs: character to be used for the right side of the window
             * 4. ts: character to be used for the top side of the window
             * 5. bs: character to be used for the bottom side of the window
             * 6. tl: character to be used for the top left corner of the window
             * 7. tr: character to be used for the top right corner of the window
             * 8. bl: character to be used for the bottom left corner of the window
             * 9. br: character to be used for the bottom right corner of the window
             */
            wrefresh(p);
            delwin(p);
        });
        wrefresh(command_window.get());

        log_window = std::shared_ptr<WINDOW>(newwin(LINES - 5, COLS, 5, 0), [](auto p) {
            /* box(local_win, ' ', ' '); : This won't produce the desired
             * result of erasing the window. It will leave it's four corners
             * and so an ugly remnant of window.
             */
            wborder(p, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
            /* The parameters taken are
             * 1. win: the window on which to operate
             * 2. ls: character to be used for the left side of the window
             * 3. rs: character to be used for the right side of the window
             * 4. ts: character to be used for the top side of the window
             * 5. bs: character to be used for the bottom side of the window
             * 6. tl: character to be used for the top left corner of the window
             * 7. tr: character to be used for the top right corner of the window
             * 8. bl: character to be used for the bottom left corner of the window
             * 9. br: character to be used for the bottom right corner of the window
             */
            wrefresh(p);
            delwin(p);
        });
        scrollok(log_window.get(), true);
        wrefresh(log_window.get());
    }

    void KeyboardWalk::update_window(const std::shared_ptr<WINDOW>& window,
                                     const LogColours& colours,
                                     const std::string& source,
                                     const std::string& message,
                                     const bool& print_level) {
        std::lock_guard<std::mutex> lock(mutex);

        // Print the message source
        wprintw(window.get(), source.c_str());

        // Print the log level if it is enabled
        if (print_level) {
            // Print it in colour if the functionality is available
            if (colours_enabled) {
                wattron(window.get(), COLOR_PAIR(short(colours)));
            }
            switch (colours) {
                case LogColours::TRACE_COLOURS: wprintw(window.get(), "TRACE: "); break;
                case LogColours::DEBUG_COLOURS: wprintw(window.get(), "DEBUG: "); break;
                case LogColours::INFO_COLOURS: wprintw(window.get(), "INFO: "); break;
                case LogColours::WARN_COLOURS: wprintw(window.get(), "WARN: "); break;
                case LogColours::ERROR_COLOURS: waddwstr(window.get(), L"(╯°□°）╯︵ ┻━┻: "); break;
                case LogColours::FATAL_COLOURS: waddwstr(window.get(), L"(ノಠ益ಠ)ノ彡┻━┻: "); break;
            }
            if (colours_enabled) {
                wattroff(window.get(), COLOR_PAIR(short(colours)));
            }
        }

        // Print the log message and refresh the screen
        wprintw(window.get(), "%s\n", message.c_str());
        wrefresh(window.get());
    }

    void KeyboardWalk::forward() {
        velocity.x() += DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("forward");
    }

    void KeyboardWalk::left() {
        velocity.y() += DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("left");
    }

    void KeyboardWalk::back() {
        velocity.x() -= DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("back");
    }

    void KeyboardWalk::right() {
        velocity.y() -= DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("right");
    }

    void KeyboardWalk::turn_left() {
        rotation += ROT_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("turn left");
    }

    void KeyboardWalk::turn_right() {
        rotation -= ROT_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("turn right");
    }

    void KeyboardWalk::get_up() {
        update_command();
        print_status();
        log<NUClear::INFO>("getup");
    }

    void KeyboardWalk::kick(LimbID::Value l) {
        message::motion::KickScriptCommand ks;
        ks.leg          = l;
        std::string leg = (l == 1) ? "left" : "right";
        ks.type         = message::motion::KickCommandType::NORMAL;
        emit(std::make_unique<message::motion::KickScriptCommand>(ks));
        log<NUClear::INFO>(fmt::format("kick {}", leg));
    }

    void KeyboardWalk::look_left() {
        head_yaw += HEAD_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("look left");
    }

    void KeyboardWalk::look_right() {
        head_yaw -= HEAD_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("look right");
    }

    void KeyboardWalk::look_up() {
        head_pitch += HEAD_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("look up");
    }

    void KeyboardWalk::look_down() {
        head_pitch -= HEAD_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("look down");
    }

    void KeyboardWalk::walk_toggle() {
        if (moving) {
            emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
            moving = false;
        }
        else {
            moving = true;
            update_command();
        }
        print_status();
    }

    void KeyboardWalk::reset() {
        velocity   = Eigen::Vector2f::Zero();
        rotation   = 0.0f;
        head_yaw   = 0.0f;
        head_pitch = 0.0f;
        update_command();
        print_status();
        log<NUClear::INFO>("reset");
    }

    void KeyboardWalk::update_command() {
        if (moving) {
            Eigen::Isometry2d affineParameter;
            affineParameter.linear()      = Eigen::Rotation2Dd(rotation).toRotationMatrix();
            affineParameter.translation() = Eigen::Vector2d(velocity.x(), velocity.y());
            emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(affineParameter)));
        }

        auto head_command         = std::make_unique<HeadCommand>();
        head_command->yaw         = head_yaw;
        head_command->pitch       = head_pitch;
        head_command->robot_space = true;
        emit(head_command);
    }

    void KeyboardWalk::print_status() {
        // Clear the command window and move to top-left corner
        wmove(command_window.get(), 0, 0);
        wclear(command_window.get());
        werase(command_window.get());

        // Construct the log command message
        std::string message =
            fmt::format("Velocity: {:.4f}, {:.4f}\nRotation: {:.4f}\nMoving: {}\nHead Yaw: {:.2f}, Head Pitch: {:.2f}",
                        velocity.x(),
                        velocity.y(),
                        rotation,
                        moving,
                        head_yaw * 180.0f / float(M_PI),
                        head_pitch * 180.0f / float(M_PI));

        // Update the command window
        update_window(command_window, LogColours::TRACE_COLOURS, "", message, false);
    }

}  // namespace module::behaviour::strategy
