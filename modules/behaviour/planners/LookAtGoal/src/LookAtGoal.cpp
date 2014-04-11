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

#include "LookAtGoal.h"

#include "messages/vision/VisionObjects.h"
#include "messages/behaviour/LookStrategy.h"

namespace modules {
    namespace behaviour {
        namespace planners {

            using messages::vision::Goal;
            using messages::behaviour::LookAtAngle;

            LookAtGoal::LookAtGoal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

                on<Trigger<Every<30,Per<std::chrono::seconds>>>, With<std::vector<Goal>>>([this] (const time_t& now, const std::vector<Goal>& goals) {
                    if (goals.size() > 0) {
                        emit(std::make_unique<LookAtAngle>(LookAtAngle {goals[0].screenAngular[0],goals[0].screenAngular[0]}));
                    }

                });
            }
        }  // tools
    }  // behaviours
}  // modules
