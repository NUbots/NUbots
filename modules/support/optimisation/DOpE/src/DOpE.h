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

#ifndef MODULES_SUPPORT_OPTIMISATION_DOPE_H
#define MODULES_SUPPORT_OPTIMISATION_DOPE_H

#include <nuclear>

#include "messages/support/optimisation/DOpE.h"
#include "utility/math/optimisation/Optimiser.h"
#include "messages/support/optimisation/proto/Episode.pb.h"

namespace modules {
namespace support {
namespace optimisation {

    class DOpE : public NUClear::Reactor {
    private:
        struct Optimisation {
            bool network;
            std::unique_ptr<utility::math::optimisation::Optimiser> optimiser;
            std::vector<messages::support::optimisation::proto::Episode> episodes;
        };

        std::map<std::string, Optimisation> optimisations;

    public:
        /// @brief Called by the powerplant to build and setup the DOpE reactor.
        explicit DOpE(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
}

#endif  // MODULES_SUPPORT_OPTIMISATION_DOPE_H
