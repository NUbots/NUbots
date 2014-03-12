/*
 * This file is part of KFBallLocalisation.
 *
 * KFBallLocalisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KFBallLocalisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with KFBallLocalisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_KFBALLLOCALISATION_H
#define MODULES_LOCALISATION_KFBALLLOCALISATION_H

#include <nuclear>
#include <armadillo>
#include "KFBallLocalisationEngine.h"

namespace modules {
namespace localisation {

    class KFBallLocalisation : public NUClear::Reactor {
    private:
        /// The engine that does all of the work
        localisation::KFBallLocalisationEngine engine_;

        /// For testing
        arma::vec2 marker_ = { 0, 0 };

    public:
        /// @brief Called by the powerplant to build and setup the KFBallLocalisation reactor.
        explicit KFBallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
#endif

