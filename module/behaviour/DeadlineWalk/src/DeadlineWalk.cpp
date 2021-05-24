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

#include "DeadlineWalk.hpp"

#include <clocale>
#include <csignal>
#include <cstdio>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "message/behaviour/MotionCommand.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour::strategy {

    using message::behaviour::MotionCommand;
    using LimbID = utility::input::LimbID;

    // void quit() {
    //     std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
    // }

    KeyboardWalk::KeyboardWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), velocity(Eigen::Vector2f::Zero()) {

        on<Configuration>("Deadline.yaml").then("Empty deadline config", [this](const Configuration& config) {
            forward();
            forward();
            forward();
            moving = true;
            update_command();
        });
    }

    void KeyboardWalk::forward() {
        velocity.x() += DIFF;
        update_command();
        print_status();
        log<NUClear::INFO>("forward");
    }

    // void KeyboardWalk::get_up() {
    //     update_command();
    //     print_status();
    //     log<NUClear::INFO>("getup");
    // }

    void KeyboardWalk::update_command() {
        if (moving) {
            Eigen::Affine2d affineParameter;
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

}  // namespace module::behaviour::strategy
