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

#ifndef MODULES_LOCALISATION_H
#define MODULES_LOCALISATION_H

#include <nuclear>
#include "LocalisationEngine.h"

namespace modules {
    class Localisation : public NUClear::Reactor {
    private:
    	LocalisationEngine engine;

    public:
    	/// @brief Our configuration file for this class
        static constexpr const char* CONFIGURATION_PATH = "Localisation.json";

        /// @brief Called by the powerplant to build and setup our CameraReader
        explicit Localisation(std::unique_ptr<NUClear::Environment> environment);
    };
}
#endif

