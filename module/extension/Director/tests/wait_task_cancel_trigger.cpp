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

#include <catch2/catch_test_macros.hpp>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct SimpleTask {};
    struct SimpleMessage {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 3> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<SimpleTask>, Optional<Trigger<SimpleMessage>>>().then([this](const RunReason& run_reason) {
                if (run_reason == RunReason::OTHER_TRIGGER) {
                    events.push_back("task executed through trigger");
                }
                else if (run_reason == RunReason::NEW_TASK) {
                    events.push_back("new task executed, waiting");

                    // Rerun the provider after 100ms with Wait
                    emit<Task>(std::make_unique<Wait>(NUClear::clock::now() + std::chrono::milliseconds(100)));
                }
                else {
                    events.push_back("task executed, done waiting");
                }
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Freeze time
                emit(std::make_unique<NUClear::message::TimeTravel>(NUClear::clock::now(),
                                                                    0.0,
                                                                    NUClear::message::TimeTravel::Action::RELATIVE));

                // Emit a simple task which will emit a Wait
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());
            });

            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a SimpleMessage to trigger the SimpleTask provider
                events.push_back("emitting simple message");
                emit(std::make_unique<SimpleMessage>());

                // Advance time to when Wait would occur
                emit(std::make_unique<NUClear::message::TimeTravel>(
                    NUClear::clock::now() + std::chrono::milliseconds(100),
                    0.0,
                    NUClear::message::TimeTravel::Action::RELATIVE));
            });

            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit SimpleTask again to cause a wait
                events.push_back("emitting simple task");
                emit<Task>(std::make_unique<SimpleTask>());

                // Advance time to when Wait should finish
                emit(std::make_unique<NUClear::message::TimeTravel>(
                    NUClear::clock::now() + std::chrono::milliseconds(100),
                    0.0,
                    NUClear::message::TimeTravel::Action::RELATIVE));
            });
        }
    };


}  // namespace

TEST_CASE("Test that a Wait task will cause a provider to run again", "[director][wait][trigger]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<NUClear::extension::ChronoController>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting simple task",
                                         "new task executed, waiting",
                                         "emitting simple message",
                                         "task executed through trigger",
                                         "emitting simple task",
                                         "new task executed, waiting",
                                         "task executed, done waiting"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
