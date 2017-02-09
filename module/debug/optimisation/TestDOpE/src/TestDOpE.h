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

#ifndef MODULES_DEBUG_OPTIMISATION_TESTDOPE_H
#define MODULES_DEBUG_OPTIMISATION_TESTDOPE_H

#include <nuclear>
#include "message/support/optimisation/DOpE.h"

namespace module {
namespace debug {
namespace optimisation {

    class TestDOpE : public NUClear::Reactor {
    private:
        message::support::optimisation::Parameters currentParameters;

    public:
        /// @brief Called by the powerplant to build and setup the TestDOpE reactor.
        explicit TestDOpE(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULES_DEBUG_OPTIMISATION_TESTDOPE_H
