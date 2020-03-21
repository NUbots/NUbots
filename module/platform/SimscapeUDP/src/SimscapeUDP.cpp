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
 * Copyright 2020 NUbots <nubots@nubots.net>
 */

#include "SimscapeUDP.h"

#include "extension/Configuration.h"
#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"

namespace module {
namespace platform {
    using extension::Configuration;
    using NUClear::dsl::word::UDP;

    SimscapeUDP::SimscapeUDP(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SimscapeUDP.yaml").then([this](const Configuration& cfg) {
            config.ip      = cfg["ip"].as<std::string>();
            config.inPort  = cfg["inPort"];
            config.outPort = cfg["outPort"];
        });

        /*TEST*/
        on<NUClear::dsl::word::Every<2, std::chrono::seconds>>().then("", [this]() {
            log("sending test data");
            message::motion::ServoTarget testData;
            testData.time     = NUClear::clock::now();
            testData.id       = 0;
            testData.position = 3;
            testData.gain     = 3;
            testData.torque   = 3;
            emit(std::make_unique<message::motion::ServoTarget>(testData));
        });

        /*
         ***************************
         * PACKETS FROM SIM TO ROBOT
         ***************************
         */


        /*
         * Receive a packet from the sim and emit it for the robot
         */
        on<UDP>(config.inPort).then([this](const UDP::Packet packet) {
            log("UDP message received");
            log(packet.payload.data());

            if (packet.valid) {
                std::string data = std::string(packet.payload.begin(), packet.payload.end());

                message::platform::darwin::DarwinSensors sensors =
                    NUClear::util::serialise::Serialise<message::platform::darwin::DarwinSensors>::deserialise(
                        packet.payload);
                emit(std::make_unique<message::platform::darwin::DarwinSensors>(sensors));
            }
        });

        /*
         ***************************
         * EMISSIONS FROM ROBOT TO SIM
         ***************************
         */

        /*
         * Send updated servo positions to the sim
         */
        on<Trigger<message::motion::ServoTarget>>().then([this](const message::motion::ServoTarget& command) {
            log("Converting servotarget to servotargets and sending on");
            log(config.ip);
            log(config.outPort);

            message::motion::ServoTargets commands;

            commands.targets.push_back(command);

            emit<Scope::UDP>(std::make_unique<message::motion::ServoTargets>(commands), config.ip, config.outPort);
        });

        /*on<UDP>(config.outPort).then([this](const UDP::Packet packet) {
            log("UDP message received");
            message::motion::ServoTargets message =
                NUClear::util::serialise::Serialise<message::motion::ServoTargets>::deserialise(packet.payload);

            log(message.targets.front().id);
        });*/

        /*
         * Send updated servo positions to the sim
         */
        on<Trigger<std::vector<message::motion::ServoTarget>>>().then(
            [this](const std::vector<message::motion::ServoTarget>& commandsVector) {
                std::unique_ptr<message::motion::ServoTargets> commands =
                    std::make_unique<message::motion::ServoTargets>();

                for (const message::motion::ServoTarget& command : commandsVector) {
                    commands->targets.push_back(command);
                }

                emit<Scope::UDP>(commands, config.ip, config.outPort);
            });

        /*
         * Send updated servo positions to the sim
         */
        on<Trigger<message::motion::ServoTargets>>().then([this](const message::motion::ServoTargets& commands) {
            emit<Scope::UDP>(std::make_unique<message::motion::ServoTargets>(commands), config.ip, config.outPort);
        });
    }
}  // namespace platform
}  // namespace module
