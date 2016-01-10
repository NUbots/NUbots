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

#ifndef MODULES_PLATFORM_DARWIN_STROBEDARWIN_H
#define MODULES_PLATFORM_DARWIN_STROBEDARWIN_H

#include <nuclear>

namespace module {
    namespace platform {
        namespace darwin {

            /**
             * Makes the Darwin's eyes flash to a beat (will flash every beat)
             *
             * @author Jake Woods
             */
            class StrobeDarwin : public NUClear::Reactor {
            public:
                explicit StrobeDarwin(std::unique_ptr<NUClear::Environment> environment);
            };

        }  // darwin
    }  // platform
}  // modules

#endif  // MODULES_PLATFORM_DARWIN_STROBEDARWIN_H

