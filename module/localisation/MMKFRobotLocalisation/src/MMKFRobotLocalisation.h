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

#ifndef MODULES_LOCALISATION_MMKFROBOTLOCALISATION_H
#define MODULES_LOCALISATION_MMKFROBOTLOCALISATION_H

#include <nuclear>
#include <armadillo>
#include "MMKFRobotLocalisationEngine.h"

namespace module {
namespace localisation {
    class MMKFRobotLocalisation : public NUClear::Reactor {
    private:
        /// For testing
        arma::vec2 marker_ = { 0, 0 };
        /// The engine that does all of the work
        std::unique_ptr<localisation::MMKFRobotLocalisationEngine> engine_;

        //Disable until first data
        ReactionHandle emit_data_handle;

        NUClear::clock::time_point last_measurement_time;

        void graphMMRMHypotheses(const std::string& descr, MultiModalRobotModel& mmrm);

    public:
        /// @brief Called by the powerplant to build and setup the MMKFRobotLocalisation reactor.
        explicit MMKFRobotLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}
#endif

