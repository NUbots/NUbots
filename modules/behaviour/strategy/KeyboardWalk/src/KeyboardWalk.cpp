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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KeyboardWalk.h"

#include <ncurses.h>
#include <csignal>
#include <format.h>

#include "messages/behaviour/MotionCommand.h"
#include "messages/motion/HeadCommand.h"
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/Action.h"

namespace modules {
namespace behaviour {
namespace strategy {

    using messages::behaviour::MotionCommand;
    using messages::motion::HeadCommand;
    using messages::motion::KickCommand;
    using messages::input::LimbID;

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        velocity.zeros();

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&KeyboardWalk::run), this), std::bind(std::mem_fn(&KeyboardWalk::kill), this)));

        // emit<Scope::INITIALIZE>(std::make_unique<WalkStartCommand>());
        // moving = true;
    }

    void KeyboardWalk::run() {
        // TODO: on command change emit velocity changes
        // TODO: on fall, use get up?
        // TODO: kill motors command

        // Start curses mode
        initscr();
        // Capture our characters immediately (but pass through signals)
        cbreak();
        // Capture arrows and function keys
        keypad(stdscr, true);
        // Don't echo the users messages
        noecho();
        // Hide the cursor
        //curs_set(false);

        on<Trigger<NUClear::LogMessage>, Options<Sync<KeyboardWalk>>>([this](const NUClear::LogMessage& message) {
            printw((message.message + "\n").c_str());
            refresh();
        });

        printStatus();

        do {
            switch (tolower(getch())) {
                case 'w':
                    forward();
                    break;
                case 'a':
                    left();
                    break;
                case 's':
                    back();
                    break;
                case 'd':
                    right();
                    break;
                case 'z':
                    turnLeft();
                    break;
                case 'x':
                    turnRight();
                    break;
                case 'r':
                    reset();
                    break;
                case 'g':
                    getUp();
                    break;
                case 'e':
                    walkToggle();
                    break;
                case ' ':
                    kickRightForward();
                    break;
                case KEY_LEFT:
                    lookLeft();
                    break;
                case KEY_RIGHT:
                    lookRight();
                    break;
                case KEY_UP:
                    lookUp();
                    break;
                case KEY_DOWN:
                    lookDown();
                    break;
                case 'q':
                    quit();
                    return;
                default:
                    log("Unknown Command");
            }
        } while (running);
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

    void KeyboardWalk::kickRightForward() {
        emit(std::make_unique<KickCommand>(KickCommand{
            {-0.05, 0, 0}, //Ball is right of centre for right kick
            {1, 0, 0}
        }));
        log("right forward kick");
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
        headPitch -= HEAD_DIFF;
        updateCommand();
        printStatus();
        log("look up");
    }

    void KeyboardWalk::lookDown() {
        headPitch += HEAD_DIFF;
        updateCommand();
        printStatus();
        log("look down");
    }

    void KeyboardWalk::walkToggle() {
        if (moving) {
            auto motionCommand = std::make_unique<MotionCommand>();
            motionCommand->type = MotionCommand::Type::StandStill;
            emit(std::move(motionCommand));
            moving = false;
        } else {
            moving = true;
            updateCommand();
        }
        printStatus();
    }

    void KeyboardWalk::reset() {
        velocity = {0, 0};
        rotation = 0;
        headYaw = 0;
        headPitch = 0;
        updateCommand();
        printStatus();
        log("reset");
    }

    void KeyboardWalk::updateCommand() {
        if (moving) {
            auto motionCommand = std::make_unique<MotionCommand>();
            motionCommand->type = MotionCommand::Type::DirectCommand;
            motionCommand->walkCommand.xy() = velocity;
            motionCommand->walkCommand.angle() = rotation;
            emit(std::move(motionCommand));
        }

        auto headCommand = std::make_unique<HeadCommand>();
        headCommand->yaw = headYaw;
        headCommand->pitch = headPitch;
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
        std::raise(SIGINT);
    }

    void KeyboardWalk::kill() {
        running = false;
    }

}
}
}
