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

#include "NUsightHarness.h"

#include "utility/nusight/NUhelpers.h"

namespace module {
namespace support {

    using NUClear::DEBUG;
    using std::chrono::milliseconds;
    using utility::nusight::graph;

    NUsightHarness::NUsightHarness(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

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
    }

}  // namespace support
}  // namespace module
