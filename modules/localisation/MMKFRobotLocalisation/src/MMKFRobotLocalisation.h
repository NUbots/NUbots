/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_MMKFROBOTLOCALISATION_H
#define MODULES_LOCALISATION_MMKFROBOTLOCALISATION_H

#include <nuclear>
#include <armadillo>
#include "MMKFRobotLocalisationEngine.h"
#include "FieldDescription.h"

namespace modules {
namespace localisation {

    class MMKFRobotLocalisation : public NUClear::Reactor {
    private:
        /// The engine that does all of the work
        localisation::MMKFRobotLocalisationEngine engine_;

        /// For testing
        arma::vec2 marker_ = { 0, 0 };

    public:
        /// @brief General localisation configuration.
        struct LocalisationConfig {
            static constexpr const char* CONFIGURATION_PATH = "Localisation.json";
        };

        /// @brief Called by the powerplant to build and setup the Localisation reactor.
        explicit MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
#endif

