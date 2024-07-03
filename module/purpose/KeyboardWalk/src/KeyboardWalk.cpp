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
#include "KeyboardWalk.hpp"

#include <clocale>
#include <csignal>
#include <cstdio>
#include <fmt/format.h>
#include <string>
#include <termios.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Buttons.hpp"
#include "message/localisation/Field.hpp"
#include "message/output/Buzzer.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/StartSafely.hpp"

namespace module::purpose {

    using extension::Configuration;
    using extension::behaviour::Task;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::ButtonMiddleDown;
    using message::input::ButtonMiddleUp;
    using message::localisation::ResetFieldLocalisation;
    using message::output::Buzzer;
    using message::skill::Kick;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;
    using message::strategy::StartSafely;
    using NUClear::message::LogMessage;
    using utility::input::LimbID;

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Configuration>("KeyboardWalk.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KeyboardWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<ButtonMiddleDown>, Single>().then([this] {
            emit<Scope::DIRECT>(std::make_unique<ResetFieldLocalisation>());
            emit<Scope::DIRECT>(std::make_unique<Buzzer>(1000));
        });

        on<Trigger<ButtonMiddleUp>, Single>().then([this] { emit<Scope::DIRECT>(std::make_unique<Buzzer>(0)); });

        // Start the Director graph for the KeyboardWalk.
        on<Startup>().then([this] {
            // At the start of the program, we should be standing
            // Without these emis, modules that need a Stability and WalkState messages may not run
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));

            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 5);

            // Start up safely with low gains
            emit<Task>(std::make_unique<StartSafely>(), 4);

            // Stand Still on startup
            // emit<Task>(std::make_unique<StandStill>());

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
            update_command();
            print_status();
        });

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

        on<Trigger<LogMessage>>().then([this](const LogMessage& message) {
            // Only display messages that are above the display level of the reactor that made the log
            if (message.level < message.display_level) {
                return;
            };
            // Where this message came from
            std::string source = "";

            // If we know where this log message came from, we display that
            if (message.task != nullptr) {
                // Get our reactor name
                std::string reactor = message.task->identifiers.reactor;

                // Strip to the last semicolon if we have one
                size_t lastC = reactor.find_last_of(':');
                reactor      = lastC == std::string::npos ? reactor : reactor.substr(lastC + 1);

                // This is our source
                source = reactor + " "
                         + (message.task->identifiers.name.empty() ? "" : "- " + message.task->identifiers.name + " ");
            }

            LogColours colours;
            switch (message.level) {
                case NUClear::TRACE: colours = LogColours::TRACE_COLOURS; break;
                case NUClear::DEBUG: colours = LogColours::DEBUG_COLOURS; break;
                case NUClear::INFO: colours = LogColours::INFO_COLOURS; break;
                case NUClear::WARN: colours = LogColours::WARN_COLOURS; break;
                case NUClear::ERROR: colours = LogColours::ERROR_COLOURS; break;
                case NUClear::UNKNOWN:;
                case NUClear::FATAL: colours = LogColours::FATAL_COLOURS; break;
            }

            update_window(log_window, colours, source, message.message, true);
        });

        on<Shutdown>().then(endwin);
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
        walk_command.x() += DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("forward");
    }

    void KeyboardWalk::left() {
        walk_command.y() += DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("left");
    }

    void KeyboardWalk::back() {
        walk_command.x() -= DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("back");
    }

    void KeyboardWalk::right() {
        walk_command.y() -= DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("right");
    }

    void KeyboardWalk::turn_left() {
        walk_command.z() += ROT_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("turn left");
    }

    void KeyboardWalk::turn_right() {
        walk_command.z() -= ROT_DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("turn right");
    }

    void KeyboardWalk::kick(LimbID::Value l) {
        reset();
        switch (l) {
            case LimbID::LEFT_LEG: emit<Task>(std::make_unique<Kick>(LimbID::LEFT_LEG), 3); break;
            case LimbID::RIGHT_LEG: emit<Task>(std::make_unique<Kick>(LimbID::RIGHT_LEG), 3); break;
            default: log<NUClear::ERROR>("Invalid limb ID");
        }
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
        if (walk_enabled) {
            walk_enabled = false;
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d::Zero()), 2);
        }
        else {
            walk_enabled = true;
            update_command();
        }
        print_status();
    }

    void KeyboardWalk::reset() {
        walk_command = Eigen::Vector3d::Zero();
        head_yaw     = 0.0f;
        head_pitch   = 0.0f;
        update_command();
        print_status();
        log<NUClear::INFO>("reset");
    }

    void KeyboardWalk::quit() {
        endwin();
        std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
    }

    void KeyboardWalk::update_command() {
        // If walking is enabled, update the walk command
        if (walk_enabled) {
            emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(walk_command.x(), walk_command.y(), walk_command.z())),
                       2);
        }

        // Create a unit vector in the direction the head should be pointing
        Eigen::Vector3d uPCt = (Eigen::AngleAxisd(head_yaw, Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(-head_pitch, Eigen::Vector3d::UnitY()))
                                   .toRotationMatrix()
                               * Eigen::Vector3d::UnitX();
        emit<Task>(std::make_unique<Look>(uPCt, false));
    }

    void KeyboardWalk::print_status() {
        // Clear the command window and move to top-left corner
        wmove(command_window.get(), 0, 0);
        wclear(command_window.get());
        werase(command_window.get());

        // Construct the log command message
        std::string message = fmt::format(
            "Velocity: {:.4f}, {:.4f}\nRotation: {:.4f}\nWalk Enabled: {}\nHead Yaw: {:.2f}, Head Pitch: {:.2f}",
            walk_command.x(),
            walk_command.y(),
            walk_command.z(),
            walk_enabled,
            head_yaw * 180.0f / float(M_PI),
            head_pitch * 180.0f / float(M_PI));

        // Update the command window
        update_window(command_window, LogColours::TRACE_COLOURS, "", message, false);
    }

}  // namespace module::purpose
