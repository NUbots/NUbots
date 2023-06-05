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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP
#define MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

template <typename BaseClass, int timeout = 1000>
class TestBase : public extension::behaviour::BehaviourReactor {
public:
    // Struct to use to emit each step of the test, by doing each step in a separate reaction with low priority, it will
    // ensure that everything has finished changing before the next step is run
    template <int i>
    struct Step {};

    struct ShutdownOnIdle {};

    explicit TestBase(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        // Timeout if the test doesn't complete in time
        on<Watchdog<BaseClass, timeout, std::chrono::milliseconds>>().then([this] {
            std::cout << "Test timed out" << std::endl;
            powerplant.shutdown();
        });

        // Shutdown if the system is idle
        on<Trigger<ShutdownOnIdle>, Priority::IDLE>().then([this] { powerplant.shutdown(); });
        on<Startup>().then([this] { emit(std::make_unique<ShutdownOnIdle>()); });
    }
};

#endif  // MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP
