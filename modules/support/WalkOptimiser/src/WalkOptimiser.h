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

#ifndef MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H
#define MODULES_SUPPORT_OPTIMISATION_WALK_OPTIMISER_H

#include <nuclear>

namespace modules {
    namespace support {
        namespace optimisation {

            class WalkOptimiser : public NUClear::Reactor {
            private:
            public:
                static constexpr const char* CONFIGURATION_PATH = "WalkScripter.yaml";
                explicit WalkOptimiser(std::unique_ptr<NUClear::Environment> environment);
            };

        } //optimisation
    }  // support
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

