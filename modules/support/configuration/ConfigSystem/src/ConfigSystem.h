/*
 * This file is part of ConfigSystem.
 *
 * ConfigSystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConfigSystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConfigSystem.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_
#define MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_

#include <nuclear>

#include "utility/idiom/pimpl.h"

namespace modules {
    namespace support {
        namespace configuration {

            /**
             * Handles configuration objects for the rest of the system.
             *
             * @author Trent Houliston
             * @author Michael Burton
             */
            class ConfigSystem : public NUClear::Reactor {

            private:
                class impl;
                ::utility::idiom::pimpl<impl> m;

           public:
                explicit ConfigSystem(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // configuration
    }  // support
}  // modules

#endif  // MODULES_UTILITY_CONFIGURATION_CONFIGSYSTEM_H_

