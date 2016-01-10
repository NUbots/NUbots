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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUcap.h"
#include <nuclear>

#include "utility/nubugger/NUhelpers.h"
#include "message/input/proto/MotionCapture.pb.h"

namespace module {
namespace debug {

    using utility::nubugger::graph;
    using message::input::proto::MotionCapture;

    NUcap::NUcap(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        /*on<Network<MotionCapture>>([this](const Network<MotionCapture>& net) {
            // log("NUcap: Network<MotionCapture> received!");
            //auto& mocap = net.data;
            //for (auto& rigidBody : mocap->rigid_bodies()) {

                // int id = rigidBody.identifier();
                // float x = rigidBody.position().x();
                // float y = rigidBody.position().y();
                // float z = rigidBody.position().z();

                // log("NUcap:", id, "x:", x, "y:", y, "z:", z);
                // emit(graph("NUcap", x, y, z));
                // TODO: transform from head to field
            //}

        });*/
    }

}
}

