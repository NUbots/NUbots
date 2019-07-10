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

#include "NUsight.h"

#include "message/localisation/Ball.h"
#include "message/localisation/Field.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {

    using message::localisation::Ball;
    using message::localisation::Field;
    using utility::nusight::graph;

    void NUsight::provideLocalisation() {

        handles["localisation"].push_back(
            on<Trigger<Field>, Single, Priority::LOW>().then([this](std::shared_ptr<const Field> self) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(self), "nusight", false);
            }));

        handles["localisation"].push_back(
            on<Trigger<Ball>, Single, Priority::LOW>().then([this](std::shared_ptr<const Ball> ball) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(ball), "nusight", false);
            }));
    }
}  // namespace support
}  // namespace module
