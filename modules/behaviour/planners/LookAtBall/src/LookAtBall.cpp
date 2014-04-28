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

#include "LookAtBall.h"

#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"
#include "messages/behaviour/LookStrategy.h"

namespace modules {
    namespace behaviour {
        namespace planners {

            using messages::vision::Ball;
            using messages::vision::Goal;
            using messages::behaviour::LookAtAngle;

            LookAtBall::LookAtBall(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                
                //this reaction focuses on the ball - pan'n'scan if not visible and focus on as many objects as possible if visible
                on<Trigger<std::vector<Ball>>,
                    With<std::vector<Goal>>,
                    With<Optional<messages::localisation::Ball>> >([this] 
                    (const std::vector<Ball>& balls,
                     const std::vector<Goal>& goals,
                     const std::shared_ptr<const messages::localisation::Ball>& ball) {
                
                    if (balls.size() > 0) {
                        std::vector<LookAtAngle> angles(4);
                        
                        angles.emplace_back(LookAtAngle {balls[0].screenAngular[0],-balls[0].screenAngular[1]});
                        
                        for (const auto& g : goals) {
                            angles.emplace_back(LookAtAngle {g.screenAngular[0],-g.screenAngular[1]});
                        }
                        
                        //XXX: add looking at robots as well
                        
                        emit(std::make_unique<std::vector<LookAtAngle>>(angles));
                    } else {
                        //XXX: do a scan'n'pan
                        //std::vector<LookAtPoint> points(8);
                    }

                });
            }
        }  // tools
    }  // behaviours
}  // modules
