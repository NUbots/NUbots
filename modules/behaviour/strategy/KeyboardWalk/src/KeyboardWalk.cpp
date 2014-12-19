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

#include "messages/motion/WalkCommand.h"

namespace modules {
namespace behaviour {
namespace strategy {

    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        velocity.zeros();

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&KeyboardWalk::run), this), std::bind(std::mem_fn(&KeyboardWalk::kill), this)));

        on<Trigger<WalkStartCommand>>([this](const WalkStartCommand&) {
            moving = true;
        });

        on<Trigger<WalkStopCommand>>([this](const WalkStopCommand&) {
            moving = false;
        });

        emit<Scope::INITIALIZE>(std::make_unique<WalkStartCommand>());
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
                case KEY_LEFT:
                    turnLeft();
                    break;
                case KEY_RIGHT:
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
        log("forward");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::left() {
        velocity[1] += DIFF;
        log("left");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::back() {
        velocity[0] -= DIFF;
        log("back");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::right() {
        velocity[1] -= DIFF;
        log("right");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::turnLeft() {
        rotation += ROT_DIFF;
        log("turn left");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::turnRight() {
        rotation -= ROT_DIFF;
        log("turn right");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::getUp() {
        log("getup");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::walkToggle() {
        if (moving) {
            emit(std::make_unique<WalkStopCommand>());
        }
        else {
            emit(std::make_unique<WalkStartCommand>());
        }
        printStatus();
    }

    void KeyboardWalk::reset() {
        velocity = {0, 0};
        rotation = 0;
        log("reset");
        updateCommand();
        printStatus();
    }

    void KeyboardWalk::updateCommand() {
        auto walkCommand = std::make_unique<WalkCommand>();
        walkCommand->velocity = velocity;
        walkCommand->rotationalSpeed = rotation;
        emit(std::move(walkCommand));
    }

    void KeyboardWalk::printStatus() {
        erase();
        log(fmt::format("Velocity: {:.4f}, {:.4f}", velocity[0], velocity[1]));
        log(fmt::format("Rotation: {:.4f}", rotation));
        log(fmt::format("Moving: {}", moving));
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
