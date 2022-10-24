/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include "Matlab.hpp"

#include <chrono>
#include <fmt/format.h>
#include <iostream>
#include <string>

#include "clock/clock.hpp"
#include "matlab_controller.hpp"

#include "extension/Configuration.hpp"

#include "message/behaviour/ServoCommand.hpp"

#include "utility/input/ServoID.hpp"

namespace module::platform {

    using extension::Configuration;

    using message::behaviour::ServoCommands;

    using utility::input::ServoID;


    Matlab::Matlab(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), subsumption_id(size_t(this) * size_t(this) - size_t(this)) {
        on<Configuration>("Matlab.yaml").then([this](const Configuration& config) {
            log_level      = config["log_level"].as<NUClear::LogLevel>();
            cfg.tcp_port   = config["TCP_PORT"];
            cfg.timestep   = config["TIMESTEP"];
            cfg.servo_gain = config["SERVO_GAIN"];
            cfg.n_servos   = config["N_SERVOS"];


            // Start you server
            log<NUClear::DEBUG>("Matlab server started");

            std::unique_ptr<MatlabServer> robot = std::make_unique<MatlabServer>(cfg.timestep, cfg.tcp_port);
            robot->run();

            // Convert the joint values into the correct format
            // auto waypoints = std::make_unique<ServoCommands>();
            // waypoints->commands.reserve(cfg.n_servos);

            // const NUClear::clock::time_point time = NUClear::clock::now() + Per<std::chrono::seconds>(cfg.timestep);

            // for (const auto& joint : joints) {
            //     waypoints->commands.emplace_back(subsumption_id, time, joint.first, joint.second, cfg.servo_gain,
            //     100);
            // }


            // Emit them to system
        });
    }

}  // namespace module::platform
