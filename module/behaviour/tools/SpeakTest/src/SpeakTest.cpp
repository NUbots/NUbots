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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "SpeakTest.h"

#include <nuclear>

#include "message/output/Say.h"

namespace module {
namespace behaviour {
    namespace tools {

        SpeakTest::SpeakTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            on<Every<30, std::chrono::seconds>>().then(
                [this] { emit(std::make_unique<message::output::Say>("Bite Me")); });
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
