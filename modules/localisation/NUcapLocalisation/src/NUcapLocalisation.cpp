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

#include "NUcapLocalisation.h"
#include "utility/nubugger/NUgraph.h"
#include "messages/input/proto/MotionCapture.pb.h"
#include "messages/input/Sensors.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include <armadillo>

namespace modules {
namespace localisation {

    using utility::nubugger::graph;
    using messages::input::proto::MotionCapture;
    using messages::input::Sensors;
    using utility::math::geometry::UnitQuaternion;

    NUcapLocalisation::NUcapLocalisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Network<MotionCapture>>, With<Sensors> >([this](const Network<MotionCapture>& net, const Sensors& sensors) {
            auto& mocap = net.data;
            for (auto& rigidBody : mocap->rigid_bodies()) {


                int id = rigidBody.identifier();
                if (id == 2) { // Robot #2
                    //TODO: switch to correct xyz coordinate system!!!!!!!!!!
                    float x = rigidBody.position().x();
                    float y = rigidBody.position().y();
                    float z = rigidBody.position().z();
                    UnitQuaternion q(arma::vec4{rigidBody.rotation().w(),
                                                rigidBody.rotation().x(),
                                                rigidBody.rotation().y(),
                                                rigidBody.rotation().z()});

                    arma::mat33 groundToWorldRotation = q.getMatrix() * sensors.orientationCamToGround.submat(0,0,2,2).t();

                    double bearing = std::acos(groundToWorldRotation(0,0));

                    // TODO: transform from head to field

                    emit(graph("NUcap pos", x, y, z));
                    emit(graph("NUcap bearing", bearing));
                }
            }

        });
    }
}
}
