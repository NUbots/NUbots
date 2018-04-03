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

#include "NUbugger.h"

#include <random>

#include "message/input/Sensors.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/input/ServoID.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
namespace debug {

    using NUClear::DEBUG;
    using message::input::Sensors;
    using message::platform::darwin::DarwinSensors;
    using std::chrono::milliseconds;
    using utility::nubugger::drawArrow;
    using utility::nubugger::drawSphere;
    using utility::nubugger::graph;
    using ServoID = utility::input::ServoID;
    using message::support::nubugger::DrawObjects;

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Every<50, milliseconds>>().then([this] {

            double period = 10;
            double freq   = 1 / period;
            double t      = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            float sine    = sin(2 * M_PI * freq * t);
            float cosine  = cos(2 * M_PI * freq * t);
            float dsine   = 2 * sine;
            float dcosine = 4 * cosine;

            emit(graph("Debug Waves", sine, cosine, dsine, dcosine));

        });

        on<Trigger<Sensors>, Single, Priority::LOW>().then([this](const Sensors& sensors) {

            emit(graph(
                "Servo " + static_cast<std::string>(static_cast<ServoID>(sensors.servo.at(ServoID::L_HIP_ROLL).id)),
                sensors.servo.at(ServoID::L_HIP_ROLL).presentPosition));
        });

        on<Every<1, std::chrono::seconds>>().then([this] {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-2, 2);
            double x = dis(gen);
            double y = dis(gen);
            double z = dis(gen);

            double period = 10;
            double freq   = 1 / period;
            double t      = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            float sine    = sin(2 * M_PI * freq * t);
            float cosine  = cos(2 * M_PI * freq * t);

            emit(drawArrow("arrow", arma::vec3({x, y, std::abs(z)}), arma::vec3({sine, cosine, 0}), sine));
            emit(drawSphere("sphere", arma::vec3({x, z, std::abs(z)}), std::abs(sine)));

        });
    }

}  // namespace debug
}  // namespace module
