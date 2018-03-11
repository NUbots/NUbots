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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUcapLocalisation.h"
#include <armadillo>
#include "extension/Configuration.h"
#include "message/input/MotionCapture.h"
#include "message/input/Sensors.h"
#include "utility/math/angle.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::input::MotionCapture;
    using message::input::Sensors;
    using message::localisation::Self;
    using utility::math::geometry::UnitQuaternion;
    using utility::math::matrix::Rotation3D;
    using utility::nubugger::graph;

    NUcapLocalisation::NUcapLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("NUcapLocalisation.yaml").then([this](const Configuration& config) {
            robot_id = config["robot_id"].as<int>();
            NUClear::log(
                "NUcapLocalisation::robot_id = ", robot_id, ". If incorrect change config/NUcapLocalisation.yaml");
        });

        on<Network<MotionCapture>>([this](const MotionCapture& mocap) {

            for (auto& rigidBody : mocap.rigid_bodies()) {

                int id = rigidBody.identifier();
                if (id == robot_id) {
                    // TODO: switch to correct xyz coordinate system!!!!!!!!!!
                    float x = rigidBody.position().x();
                    float y = rigidBody.position().y();
                    float z = rigidBody.position().z();
                    UnitQuaternion q(arma::vec4{rigidBody.rotation().x(),
                                                rigidBody.rotation().y(),
                                                rigidBody.rotation().z(),
                                                rigidBody.rotation().t()});

                    Rotation3D groundToWorldRotation = q;  // * sensors.camToGround.submat(0,0,2,2).t();

                    double heading = utility::math::angle::acos_clamped(groundToWorldRotation(0, 0));

                    // TODO: transform from head to field
                    auto selfs = std::make_unique<std::vector<Self>>();
                    selfs->push_back(Self());
                    selfs->back().heading  = arma::normalise(groundToWorldRotation.submat(0, 0, 1, 0));
                    selfs->back().position = arma::vec2{x, y};
                    emit(std::move(selfs));

                    emit(graph("NUcap pos", x, y, z));
                    emit(graph("NUcap heading", heading));
                }
            }

        });
    }
}  // namespace localisation
}  // namespace module
