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

#include "message/support/Configuration.h"
#include "message/input/proto/MotionCapture.pb.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"

#include <armadillo>

namespace module {
namespace support {

    using utility::math::geometry::UnitQuaternion;
    using utility::math::matrix::Rotation3D;
	using message::input::proto::MotionCapture;
    using message::support::Configuration;

    MocapRecorder::MocapRecorder(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("MocapRecorder.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file MocapRecorder.yaml
        });

        on<Trigger<MotionCapture>, Single>().then([this](const MotionCapture& mocap){

        	arma::mat rigidBodies(13,mocap.rigid_bodies().size());
        	int i = 0;
        	for (auto& rigidBody : mocap.rigid_bodies()) {

                int id = rigidBody.identifier();
                float x = rigidBody.position().x();
                float y = rigidBody.position().y();
                float z = rigidBody.position().z();
                UnitQuaternion q(arma::vec4{rigidBody.rotation().t(),
                							rigidBody.rotation().x(),
                                            rigidBody.rotation().y(),
                                            rigidBody.rotation().z()
                                            });
                Rotation3D r(q);
                rigidBodies.col(i++) = arma::vec({id,x,y,z, r.row(0)[0],r.row(0)[1],r.row(0)[2],
	                										r.row(1)[0],r.row(1)[1],r.row(1)[2],
	                										r.row(2)[0],r.row(2)[1],r.row(2)[2]});

            }
            auto now = NUClear::clock::now();
            std::stringstream filename;
            filename << "mocapdata/" << std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
            log("Saving MotionCapture data to ",filename.str());
            bool success = rigidBodies.save(filename.str());
            // log(success ? "Saved Successfully" : "Save FAILED!!!");
        });
    }
}
}
