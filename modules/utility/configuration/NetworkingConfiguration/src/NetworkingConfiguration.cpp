/*
 * This file is part of Networking Configuration.
 *
 * Networking Configuration is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Networking Configuration is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Networking Configuration.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NetworkingConfiguration.h"
#include "messages/Configuration.h"

namespace modules {

    NetworkingConfiguration::NetworkingConfiguration(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<messages::Configuration<NetworkingConfiguration>>>([this]
                (const messages::Configuration<NetworkingConfiguration>& config) {
            auto c = std::make_unique<NUClear::extensions::NetworkingConfiguration>();
            
            // Put our configuration options into a NUClear config object
            std::string deviceName = config.config["deviceName"];
            std::string networkAddress = config.config["networkAddress"];
            
            c->deviceName = deviceName;
            c->networkAddress = networkAddress;
            
            // Send the config now
            emit<Scope::DIRECT>(std::move(c));
        });
    }
}
