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
#include "utility/nubugger/NUhelpers.h"
#include "messages/input/proto/MotionCapture.pb.h"
#include "messages/input/Sensors.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include <armadillo>

namespace modules {
namespace localisation {

    using utility::nubugger::graph;
    using messages::input::proto::MotionCapture;
    using messages::input::Sensors;
    using utility::math::geometry::UnitQuaternion;
    using messages::localisation::Self;
    using messages::support::Configuration;

    NUcapLocalisation::NUcapLocalisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Configuration<NUcapLocalisation>>>([this](const Configuration<NUcapLocalisation>& config){
            robot_id = config["robot_id"].as<int>();
            NUClear::log("NUcapLocalisation::robot_id = ", robot_id, ". If incorrect change config/NUcapLocalisation.yaml");
        });

        on<Trigger<Network<MotionCapture>>, With<Sensors> >([this](const Network<MotionCapture>& net, const Sensors&) {

            auto& mocap = net.data;
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

                    arma::mat33 groundToWorldRotation = q.getMatrix();// * sensors.orientationCamToGround.submat(0,0,2,2).t();

                    double heading = std::acos(groundToWorldRotation(0,0));

                    // TODO: transform from head to field
                    auto selfs = std::make_unique<std::vector<Self>>();
                    selfs->push_back(Self());
                    selfs->back().heading = arma::normalise(groundToWorldRotation.submat(0,0,1,0));
                    selfs->back().position = arma::vec2{x,y};
                    emit(std::move(selfs));

                    emit(graph("NUcap pos", x, y, z));
                    emit(graph("NUcap heading", heading));
                }
            }

        });
    }
}
}
