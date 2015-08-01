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

#include "MocapRecorder.h"

#include "messages/support/Configuration.h"
#include "messages/input/proto/MotionCapture.pb.h"


namespace modules {
namespace support {

	using messages::input::proto::MotionCapture;
    using messages::support::Configuration;

    MocapRecorder::MocapRecorder(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<Configuration<MocapRecorder>>>([this] (const Configuration<MocapRecorder>& config) {
            // Use configuration here from file MocapRecorder.yaml
        });

        on<Trigger<MotionCapture>, Options<Single>>([this](const MotionCapture& mocap){
        	
        	for (auto& rigidBody : mocap->rigid_bodies()) {

                int id = rigidBody.identifier();
                if (id == robot_id) {
                    //TODO: switch to correct xyz coordinate system!!!!!!!!!!
                    float x = rigidBody.position().x();
                    float y = rigidBody.position().y();
                    float z = rigidBody.position().z();
                    UnitQuaternion q(arma::vec4{rigidBody.rotation().x(),
                                                rigidBody.rotation().y(),
                                                rigidBody.rotation().z(),
                                                rigidBody.rotation().t()});

                }
            }
        	//TODO open file and print mocap to file in some format
        });
    }
}
}
