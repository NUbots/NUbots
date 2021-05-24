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

#include "extension/Configuration.hpp"

#include "message/behaviour/MotionCommand.hpp"
#include "message/motion/KinematicsModel.hpp"

#include "utility/behaviour/MotionCommand.hpp"
#include "utility/input/LimbID.hpp"

namespace module::behaviour {

    using extension::Configuration;
    using message::behaviour::MotionCommand;
    using message::motion::KinematicsModel;
    using LimbID = utility::input::LimbID;

    // void quit() {
    //     std::raise(SIGTERM);  // Change back to SIGINT if required by NUbots messaging system//
    // }

    DeadlineWalk::DeadlineWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), velocity(Eigen::Vector2f::Zero()) {
        on<Trigger<KinematicsModel>>().then([this] {
            velocity.x() = 4 * DIFF;
            update_command();
        });
    }


    void DeadlineWalk::update_command() {
        Eigen::Affine2d affineParameter = Eigen::Affine2d::Identity();
        affineParameter.translation()   = Eigen::Vector2d(velocity.x(), 0.0);
        emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(affineParameter)));
    }

}  // namespace module::behaviour
