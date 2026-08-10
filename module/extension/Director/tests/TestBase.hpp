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

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <nuclear>
#include <string>

#include "extension/Behaviour.hpp"

/**
 * A base class for tests that run a series of steps.
 *
 * This class is a BehaviourReactor that will run a series of steps in a test.
 * Each step is executed in turn once the system has become Idle after processing all the previous steps.
 *
 * At the end of the steps once the system is idle it will shutdown the powerplant.
 */
template <typename BaseClass, int NSteps>
class TestBase : public extension::behaviour::BehaviourReactor {
public:
    // Struct to use to emit each step of the test, by doing each step in a separate reaction with it will
    // ensure that everything has finished changing before the next step is run
    template <int i>
    struct Step {};

    /**
     * Emit this struct to fail the test
     */
    struct Fail {
        explicit Fail(std::string message) : message(std::move(message)) {}
        std::string message;
    };

    /**
     * Construct the test base with the given environment and timeout.
     *
     * @param environment The environment to run the test in
     * @param timeout The time to wait for the test to complete before failing
     */
    explicit TestBase(std::unique_ptr<NUClear::Environment> environment,
                      bool auto_shutdown                          = true,
                      std::chrono::steady_clock::duration timeout = std::chrono::milliseconds(1000))
        : BehaviourReactor(std::move(environment)) {
        // Advance to the next step when the system is idle
        on<Idle<>>().then([this, auto_shutdown] { next_step<NSteps>(++step, auto_shutdown); });

        // Note when the system shuts down cleanly so the timeout thread can stop waiting
        on<Shutdown>().then([this] {
            const std::lock_guard<std::mutex> lock(timeout_mutex);
            clean_shutdown = true;
            timeout_cv.notify_all();
        });

        // Timeout if the test doesn't complete in time.
        // A watchdog (emit<Scope::DELAY>) cannot be used here because it rides NUClear::clock, which
        // these tests freeze and manipulate via Time Travel, so the deadline would never be reached.
        // Instead use an on<Always> reaction, which NUClear runs on its own dedicated thread (so this
        // blocking wait never stalls the default pool), to wait on a steady_clock timeout and fail the
        // test if the system has not shut down cleanly by then.
        on<Always>().then("Test Timeout", [this, timeout] {
            std::unique_lock<std::mutex> lock(timeout_mutex);
            // Wait until a clean shutdown is signalled or the timeout elapses. Checking clean_shutdown
            // through the predicate keeps the read under timeout_mutex, avoiding a race with the
            // on<Shutdown> handler that writes it.
            if (!timeout_cv.wait_for(lock, timeout, [this] { return clean_shutdown; })) {
                emit<Scope::INLINE>(std::make_unique<Fail>("Test did not complete successfully"));
                powerplant.shutdown(true);
            }
        });

        on<Trigger<Fail>, MainThread>().then([this](const Fail& f) {
            INFO(f.message);
            CHECK(false);
            powerplant.shutdown(true);
        });
    }

private:
    /**
     * Run the step with the given index.
     *
     * This function will emit the next step in the test, or shutdown the powerplant if the last step has been reached.
     * It is a recursive template function that will call itself with the next step index until the last step is
     * reached.
     *
     * Since the function has a constexpr if, the compiler should be able to optimise this into it's most efficient
     * form.
     *
     * @tparam i The index of the step to run
     *
     * @param v The index of the current step to check against
     * @param auto_shutdown If true the powerplant will shutdown after the last step
     */
    template <int I>
    void next_step(const int& v, bool auto_shutdown) {
        if (v == I) {
            emit(std::make_unique<Step<I>>());
        }
        else {
            if constexpr (I > 1) {  // Check the next step
                next_step<I - 1>(v, auto_shutdown);
            }
            else if (auto_shutdown) {  // Shutdown after the last step
                powerplant.shutdown();
            }
        }
    }

    /// The current step of the test
    int step = 0;

    /// Mutex to wait on for the test timeout
    std::mutex timeout_mutex;
    /// Condition variable to wait on for the test timeout
    std::condition_variable timeout_cv;
    /// If the system shut down cleanly (used to stop the timeout thread from failing the test)
    bool clean_shutdown = false;
};

#endif  // MODULE_EXTENSION_DIRECTOR_TESTBASE_HPP
