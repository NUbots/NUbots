/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP
#define MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

template <typename BaseClass, bool idle_shutdown = true, int timeout = 1000>
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
        if (idle_shutdown) {
            on<Trigger<ShutdownOnIdle>, Priority::IDLE>().then([this] { powerplant.shutdown(); });
            on<Startup>().then([this] { emit(std::make_unique<ShutdownOnIdle>()); });
        }
    }
};

#endif  // MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP
