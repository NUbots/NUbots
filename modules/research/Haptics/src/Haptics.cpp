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

#include "Haptics.h"
#include "messages/motion/Script.h"
#include "messages/motion/ServoWaypoint.h"
#include "messages/research/scriptoptimizer/OptimizeScript.pb.h"
#include "messages/research/scriptoptimizer/OptimizeScriptResult.pb.h"
#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
    namespace research {

        using messages::platform::darwin::DarwinSensors;
        using messages::research::scriptoptimizer::OptimizeScript;
        using messages::research::scriptoptimizer::OptimizeScriptResult;
        using messages::motion::ExecuteScript;
        using messages::motion::AllServoWaypointsComplete;
        using messages::motion::Script;

        Haptics::Haptics(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Trigger<Configuration<Haptics> > >([this](const Configuration<Haptics>& config) {
                SURFACE_THICKNESS = config["SURFACE_THICKNESS"];
                MAX_FORCE = config["MAX_FORCE"];
                STATIC_FRICTION_COEFFICIENT = config["STATIC_FRICTION_COEFFICIENT"];
                KINETIC_FRICTION_COEFFICIENT = config["KINETIC_FRICTION_COEFFICIENT"];
                SPHERE_RADIUS = config["SPHERE_RADIUS"];
            });

            on<Trigger<Every<60, Per<std::chrono::seconds> > >, With<Sensors>, Options<Single> >([this](const time_t&, const Sensors& sensors){
                //Do force feedback

            });
           
        }
    }
}
