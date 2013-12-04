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

#ifndef MODULES_SUPPORT_CONFIGURATION_NETWORKINGCONFIGURATION_H
#define MODULES_SUPPORT_CONFIGURATION_NETWORKINGCONFIGURATION_H

#include <nuclear>

namespace modules {
    namespace support {
        namespace configuration {

            /**
             * This handles the configuration of the NUClear networking system
             * through the Config system
             *
             * @author Trent Houliston
             */
            class NetworkingConfiguration : public NUClear::Reactor {
            public:
                static constexpr const char* CONFIGURATION_PATH = "NetworkingConfiguration.json";
                explicit NetworkingConfiguration(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // configuration   
    }  // support
}  // modules

#endif  // MODULES_UTILITY_CONFIGURATION_NETWORKINGCONFIGURATION_H

