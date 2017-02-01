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

#include "NetworkConfiguration.h"

#include "extension/Configuration.h"

namespace module {
namespace support {
namespace configuration {

    using extension::Configuration;

    NetworkConfiguration::NetworkConfiguration(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkConfiguration.yaml").then([this] (const Configuration& config) {
            auto netConfig = std::make_unique<NUClear::message::NetworkConfiguration>();
            netConfig->name = config["name"].as<std::string>();
            netConfig->multicast_group = config["address"].as<std::string>();
            netConfig->multicast_port = config["port"].as<int>();
            emit<Scope::DIRECT>(netConfig);
        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& message){
            char str[INET_ADDRSTRLEN];
            uint32_t addr = htonl(message.address);
            inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN);
            log<NUClear::INFO>("Connected to", message.name, "on", str);
        });

        on<Trigger<NUClear::message::NetworkLeave>>().then([this](const NUClear::message::NetworkLeave& message){
            char str[INET_ADDRSTRLEN];
            uint32_t addr = htonl(message.address);
            inet_ntop(AF_INET, &addr, str, INET_ADDRSTRLEN);
            log<NUClear::INFO>("Disconnected from", message.name, "on", str);
        });
    }
}
}
}
