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

#include "DisplayTest.h"
#include "message/input/MotionCapture.h"
#include "message/input/Sensors.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"

using utility::nusight::graph;

namespace module {
namespace support {

    using message::input::MotionCapture;
    using message::input::Sensors;

    DisplayTest::DisplayTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // TODO: remove - just for debugging the graph
        /*on<Every<100, std::chrono::milliseconds>>([this] {

            float value = float(rand()) / RAND_MAX * 100;
            emit(graph("Debug", value));

        });*/

        //         on<Network<MotionCapture>>().then([this](const Network<MotionCapture>::Source, const
        //         Network<MotionCapture>& net) {
        // //            auto mocap = net.data;
        //             // NUClear::log("I got things from", net.sender);
        //         });

        on<Trigger<Sensors>, Single, Priority::HIGH>().then(
            [this](const Sensors& sensors) { emit(graph("world", convert(sensors.Htw))); });
    }
}  // namespace support
}  // namespace module
