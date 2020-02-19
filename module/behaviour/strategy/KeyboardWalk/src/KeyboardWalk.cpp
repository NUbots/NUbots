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
#include <ncurses.h>

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

        using message::behaviour::MotionCommand;
        using message::motion::HeadCommand;
        using message::motion::KickCommand;
        using NUClear::message::LogMessage;
        using utility::math::matrix::Transform2D;
        using LimbID = utility::input::LimbID;

        KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), velocity(arma::fill::zeros) {

            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users messages
            noecho();

            updateCommand();
            printStatus();

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
                    default: log("Unknown Command");
                }
            });

            on<Trigger<LogMessage>, Sync<KeyboardWalk>>().then([this](const LogMessage& message) {
                printw((message.message + "\n").c_str());
                refresh();
            });

            on<Shutdown>().then(endwin);
        }

        void KeyboardWalk::forward() {
            velocity[0] += DIFF;
            updateCommand();
            printStatus();
            log("forward");
        }

        void KeyboardWalk::left() {
            velocity[1] += DIFF;
            updateCommand();
            printStatus();
            log("left");
        }

        void KeyboardWalk::back() {
            velocity[0] -= DIFF;
            updateCommand();
            printStatus();
            log("back");
        }

        void KeyboardWalk::right() {
            velocity[1] -= DIFF;
            updateCommand();
            printStatus();
            log("right");
        }

        void KeyboardWalk::turnLeft() {
            rotation += ROT_DIFF;
            updateCommand();
            printStatus();
            log("turn left");
        }

        void KeyboardWalk::turnRight() {
            rotation -= ROT_DIFF;
            updateCommand();
            printStatus();
            log("turn right");
        }

        void KeyboardWalk::getUp() {
            updateCommand();
            printStatus();
            log("getup");
        }

        void KeyboardWalk::kick(LimbID::Value l) {
            message::motion::KickScriptCommand ks;
            ks.direction    = {0.05, 0, 0};
            ks.leg          = l;
            std::string leg = (l == 1) ? "left" : "right";
            emit(std::make_unique<message::motion::KickScriptCommand>(ks));
            log("kick", leg);
        }

        void KeyboardWalk::lookLeft() {
            headYaw += HEAD_DIFF;
            updateCommand();
            printStatus();
            log("look left");
        }

        void KeyboardWalk::lookRight() {
            headYaw -= HEAD_DIFF;
            updateCommand();
            printStatus();
            log("look right");
        }

        void KeyboardWalk::lookUp() {
            headPitch += HEAD_DIFF;
            updateCommand();
            printStatus();
            log("look up");
        }

        void KeyboardWalk::lookDown() {
            headPitch -= HEAD_DIFF;
            updateCommand();
            printStatus();
            log("look down");
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
            log("reset");
        }

        void KeyboardWalk::updateCommand() {
            if (moving) {
                std::cout << "New command " << velocity.t() << " " << rotation << std::endl;
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
            erase();
            log(fmt::format("Velocity: {:.4f}, {:.4f}", velocity[0], velocity[1]));
            log(fmt::format("Rotation: {:.4f}", rotation));
            log(fmt::format("Moving: {}", moving));
            log(fmt::format("Head Yaw: {:.2f}, Head Pitch: {:.2f}", headYaw * 180 / M_PI, headPitch * 180 / M_PI));
        }

        void KeyboardWalk::quit() {
            endwin();
            std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
        }
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
