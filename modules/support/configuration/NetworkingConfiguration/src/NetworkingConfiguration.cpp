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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NetworkingConfiguration.h"
#include "messages/support/Configuration.h"

namespace modules {
    namespace support {
        namespace configuration {

            using messages::support::Configuration;

            NetworkingConfiguration::NetworkingConfiguration(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

                on<Trigger<Configuration<NetworkingConfiguration>>>([this]
                        (const Configuration<NetworkingConfiguration>& config) {
                    auto c = std::make_unique<NUClear::extensions::NetworkingConfiguration>();

                    // Put our configuration options into a NUClear config object
                    std::string deviceName = config["deviceName"].as<std::string>();
                    std::string networkAddress = config["networkAddress"].as<std::string>();

                    c->deviceName = deviceName;
                    c->networkAddress = networkAddress;

                    // Send the config now
                    emit<Scope::DIRECT>(std::move(c));
                });
            }

        }  // configuration
    }  // support
}  // modules