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

#include "KeyboardWalk.h"

#include <fmt/format.h>

#include <clocale>
#include <csignal>
#include <cstdio>

#include "message/behaviour/MotionCommand.h"
#include "message/motion/HeadCommand.h"
#include "message/motion/KickCommand.h"
#include "utility/behaviour/MotionCommand.h"
#include "utility/input/LimbID.h"
#include "utility/math/matrix/Transform2D.h"

namespace module {
namespace behaviour {
    namespace strategy {

        enum class LogColours : short {
            TRACE_COLOURS = 1,
            DEBUG_COLOURS = 2,
            INFO_COLOURS  = 3,
            WARN_COLOURS  = 4,
            ERROR_COLOURS = 5,
            FATAL_COLOURS = 6
        };

        struct UpdateWindow {
            UpdateWindow(const std::shared_ptr<WINDOW>& window,
                         const std::string& source,
                         const std::string& message,
                         const LogColours& colours)
                : window(window), source(source), message(message), colours(colours), print_level(true) {}
            UpdateWindow(const std::shared_ptr<WINDOW>& window, const std::string& message, const bool& print_level)
                : window(window), source(""), message(message), print_level(print_level) {}
            std::shared_ptr<WINDOW> window;
            std::string source;
            std::string message;
            LogColours colours;
            bool print_level;
        };

        using message::behaviour::MotionCommand;
        using message::motion::HeadCommand;
        using message::motion::KickCommand;
        using NUClear::message::LogMessage;
        using utility::math::matrix::Transform2D;
        using LimbID = utility::input::LimbID;

        KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), velocity(arma::fill::zeros) {

            // Ensure UTF-8 is enabled
            std::setlocale(LC_ALL, "en_US.UTF-8");

            // Start curses mode
            initscr();

            // Initialise colours
            if (has_colors() == TRUE) {
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
                    case 'z': turnLeft(); break;
                    case 'x': turnRight(); break;
                    case 'r': reset(); break;
                    case 'g': getUp(); break;
                    case 'e': walkToggle(); break;
                    case '.': kick(LimbID::RIGHT_LEG); break;
                    case ',': kick(LimbID::LEFT_LEG); break;
                    case KEY_LEFT: lookLeft(); break;
                    case KEY_RIGHT: lookRight(); break;
                    case KEY_UP: lookUp(); break;
                    case KEY_DOWN: lookDown(); break;
                    case 'q': quit(); return;
                    default:
                        log<NUClear::ERROR>("Unknown Command");
                        printStatus();
                        break;
                }
            });

            on<Trigger<LogMessage>>().then([this](const LogMessage& packet) {
                std::lock_guard<std::mutex> lock(mutex);

                // Where this message came from
                std::string source = "";

                // If we know where this log message came from, we display that
                if (packet.task) {
                    // Get our reactor name
                    std::string reactor = packet.task->identifier[1];

                    // Strip to the last semicolon if we have one
                    size_t lastC = reactor.find_last_of(':');
                    reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                    // This is our source
                    source = reactor + " "
                             + (packet.task->identifier[0].empty() ? "" : "- " + packet.task->identifier[0] + " ");
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

                emit(std::make_unique<UpdateWindow>(log_window, source, packet.message, colours));
            });

            on<Trigger<UpdateWindow>, Sync<KeyboardWalk>>().then([this](const UpdateWindow& packet) {
                wprintw(packet.window.get(), packet.source.c_str());
                if (packet.print_level) {
                    if (colours_enabled) {
                        wattron(packet.window.get(), COLOR_PAIR(short(packet.colours)));
                    }
                    switch (packet.colours) {
                        case LogColours::TRACE_COLOURS: wprintw(packet.window.get(), "TRACE: "); break;
                        case LogColours::DEBUG_COLOURS: wprintw(packet.window.get(), "DEBUG: "); break;
                        case LogColours::INFO_COLOURS: wprintw(packet.window.get(), "INFO: "); break;
                        case LogColours::WARN_COLOURS: wprintw(packet.window.get(), "WARN: "); break;
                        case LogColours::ERROR_COLOURS: waddwstr(packet.window.get(), L"(╯°□°）╯︵ ┻━┻: "); break;
                        case LogColours::FATAL_COLOURS: waddwstr(packet.window.get(), L"(ノಠ益ಠ)ノ彡┻━┻: "); break;
                    }
                    if (colours_enabled) {
                        wattroff(packet.window.get(), COLOR_PAIR(short(packet.colours)));
                    }
                }
                wprintw(packet.window.get(), "%s\n", packet.message.c_str());
                wrefresh(packet.window.get());
            });

            on<Shutdown>().then(endwin);

            updateCommand();
            printStatus();
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

        void KeyboardWalk::forward() {
            velocity[0] += DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("forward");
        }

        void KeyboardWalk::left() {
            velocity[1] += DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("left");
        }

        void KeyboardWalk::back() {
            velocity[0] -= DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("back");
        }

        void KeyboardWalk::right() {
            velocity[1] -= DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("right");
        }

        void KeyboardWalk::turnLeft() {
            rotation += ROT_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("turn left");
        }

        void KeyboardWalk::turnRight() {
            rotation -= ROT_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("turn right");
        }

        void KeyboardWalk::getUp() {
            updateCommand();
            printStatus();
            log<NUClear::INFO>("getup");
        }

        void KeyboardWalk::kick(LimbID::Value l) {
            message::motion::KickScriptCommand ks;
            ks.direction    = {0.05, 0, 0};
            ks.leg          = l;
            std::string leg = (l == 1) ? "left" : "right";
            emit(std::make_unique<message::motion::KickScriptCommand>(ks));
            log<NUClear::INFO>(fmt::format("kick {}", leg));
        }

        void KeyboardWalk::lookLeft() {
            headYaw += HEAD_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("look left");
        }

        void KeyboardWalk::lookRight() {
            headYaw -= HEAD_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("look right");
        }

        void KeyboardWalk::lookUp() {
            headPitch += HEAD_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("look up");
        }

        void KeyboardWalk::lookDown() {
            headPitch -= HEAD_DIFF;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("look down");
        }

        void KeyboardWalk::walkToggle() {
            if (moving) {
                emit(std::make_unique<MotionCommand>(utility::behaviour::StandStill()));
                moving = false;
            }
            else {
                moving = true;
                updateCommand();
            }
            printStatus();
        }

        void KeyboardWalk::reset() {
            velocity  = {0, 0};
            rotation  = 0;
            headYaw   = 0;
            headPitch = 0;
            updateCommand();
            printStatus();
            log<NUClear::INFO>("reset");
        }

        void KeyboardWalk::updateCommand() {
            if (moving) {
                emit(std::make_unique<MotionCommand>(
                    utility::behaviour::DirectCommand(Transform2D(velocity, rotation))));
            }

            auto headCommand        = std::make_unique<HeadCommand>();
            headCommand->yaw        = headYaw;
            headCommand->pitch      = headPitch;
            headCommand->robotSpace = true;
            emit(std::move(headCommand));
        }

        void KeyboardWalk::printStatus() {
            wmove(command_window.get(), 0, 0);
            wclear(command_window.get());
            werase(command_window.get());
            emit(std::make_unique<UpdateWindow>(
                command_window, fmt::format("Velocity: {:.4f}, {:.4f}", velocity[0], velocity[1]), false));
            emit(std::make_unique<UpdateWindow>(command_window, fmt::format("Rotation: {:.4f}", rotation), false));
            emit(std::make_unique<UpdateWindow>(command_window, fmt::format("Moving: {}", moving), false));
            emit(std::make_unique<UpdateWindow>(
                command_window,
                fmt::format("Head Yaw: {:.2f}, Head Pitch: {:.2f}", headYaw * 180 / M_PI, headPitch * 180 / M_PI),
                false));
        }

        void KeyboardWalk::quit() {
            endwin();
            std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
        }
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
