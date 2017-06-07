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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "KickCommander.h"

#include <Eigen/Core>

#include "extension/Configuration.h"

#include "message/motion/KickCommand.h"

#include "utility/support/yaml_expression.h"

namespace module {
namespace behaviour {
namespace tools {

    using extension::Configuration;

    using message::motion::KickCommand;
    using KickCommandType = message::motion::KickCommandType;

    KickCommander::KickCommander(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("KickCommander.yaml").then([this] (const Configuration& config) {
            log("I'm running");
            if(!doThings){
                doThings = true;
            } else {
                emit(std::make_unique<KickCommand>(KickCommand(
                   config["target"].as<Eigen::Vector3d>(),
                   config["direction"].as<Eigen::Vector3d>(),
                   KickCommandType::NORMAL
                )));
            }

        });
    }

}
}
}
