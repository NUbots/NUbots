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

#include "FixedWalk.h"

namespace modules {
    namespace behaviour {
        namespace planner {

        	using messages::motion::WalkCommand;
			using messages::behaviour::FixedWalkCommand;
			using messages::input::Sensors;


            FixedWalk::FixedWalk(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                // on<Trigger<Configuration<FixedWalk>>>([this] (const Configuration<FixedWalk>& file){                 
                // });

                on< Trigger< Every<30, Per<std::chrono::seconds>>> , Options<Sync<FixedWalk>>, With<Sensors>>([this]("Fixed Walk Manager", const NUClear::clock::time_point& t, const Sensors& sensors){
                    //Move to next segment if necessary
                    if(t > segmentStart + walkSegments.front().duration){
                        segmentStart += walkSegments.front().duration;                        
                    	walkSegments.pop_front();
                    	if(walkSegments.empty()){
                    		emit(std::make_unique<FixedWalkFinished>());
                    		return;
                    	}
	        			beginningOrientation = sensors.orientation;
                    }
                    //Emit command
                    if(!walkSegments.empty()){
                		emit(getWalkCommand(walkSegments.front(), t-segmentStart, sensors));
                	}
                });

				on<Trigger<FixedWalkCommand>, Options<Sync<FixedWalk>>, With<Sensors> >([this](const FixedWalkCommand& command, const Sensors& sensors){
	        		if(walkSegments.empty()){
	        			segmentStart = NUClear::clock::now();
	        			beginningOrientation = sensors.orientation;
	        		}
	        		for(auto& segment: command.segments){
	        			walkSegments.push_back(segment);
	        		}
				}); 

            }

            std::unique_ptr<WalkCommand> getWalkCommand(const FixedWalkCommand::WalkSegment& segment, NUClear::clock::duration t, const Sensors& sensors){
            	double vr = segment.normalisedAngularVelocity;

            	double timeSeconds = std::chrono::duration_cast<std::chrono::seconds>(t).count() 
            	arma::vec2 directionInOriginalCoords = (curvePeriod != 0 ? utility::math::matrix::zRotationMatrix(2 * M_PI * timeSeconds / segment.curvePeriod,2) : arma::eye(2,2) ) * segment.direction;
            	arma::mat33 inverseRobotRotationSinceStart = sensors.orientation.t() * beginningOrientation;
            	arma::vec2 direction = arma::normalise(inverseRobotRotationSinceStart.submat(0,0,1,1) * directionInOriginalCoords);

            	return std::make_unique<WalkCommand>({segment.normaliseVelocity * direction, vr});
            }


        }  // skills
    }  // behaviours
}  // modules
