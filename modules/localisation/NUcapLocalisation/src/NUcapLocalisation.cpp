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
#include <armadillo>


namespace modules {
namespace localisation {

    using utility::nubugger::graph;
    using messages::input::proto::MotionCapture;

    NUcapLocalisation::NUcapLocalisation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Network<MotionCapture>>>([this](const Network<MotionCapture>& net) {
            auto& mocap = net.data;
            for (auto& rigidBody : mocap->rigid_bodies()) {


                /*int id = rigidBody.identifier();
                float x = rigidBody.location().x();
                float y = rigidBody.location().y();
                float z = rigidBody.location().z();
                if (id == 2) { // Robot #2
                    // TODO: transform from head to field
                    // emit(graph("NUcap", x, y, z));
                }*/
            }

        });
    }
}
}
