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

    template <int a, int b>
    struct SubTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask>>().then([this] {
                // Task has been executed!
                events.push_back("simple task executed");
                emit<Task>(std::make_unique<SubTask<1, 0>>());
                emit<Task>(std::make_unique<SubTask<2, 0>>());
                emit<Task>(std::make_unique<SubTask<3, 0>>());
            });

            on<Provide<SubTask<1, 0>>>().then([this] {
                events.push_back("subtask 1,0 executed");
                emit<Task>(std::make_unique<SubTask<1, 1>>());
                emit<Task>(std::make_unique<SubTask<1, 2>>());
            });
            on<Provide<SubTask<1, 1>>>().then([this] { events.push_back("subtask 1,1 executed"); });
            on<Provide<SubTask<1, 2>>>().then([this] { events.push_back("subtask 1,2 executed"); });

            on<Provide<SubTask<2, 0>>>().then([this] {
                events.push_back("subtask 2,0 executed");
                emit<Task>(std::make_unique<SubTask<2, 1>>());
                emit<Task>(std::make_unique<SubTask<2, 2>>());
            });
            on<Provide<SubTask<2, 1>>>().then([this] { events.push_back("subtask 2,1 executed"); });
            on<Provide<SubTask<2, 2>>>().then([this] { events.push_back("subtask 2,2 executed"); });

            on<Provide<SubTask<3, 0>>>().then([this] {
                events.push_back("subtask 3,0 executed");
                emit<Task>(std::make_unique<SubTask<3, 1>>());
                emit<Task>(std::make_unique<SubTask<3, 2>>());
            });
            on<Provide<SubTask<3, 1>>>().then([this] { events.push_back("subtask 3,1 executed"); });
            on<Provide<SubTask<3, 2>>>().then([this] { events.push_back("subtask 3,2 executed"); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting task");
                emit<Task>(std::make_unique<SimpleTask>());
            });
            on<Startup>().then([this] {  //
                emit(std::make_unique<Step<1>>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that a tree of simple tasks are executed through the director", "[director][simple]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task",
        "simple task executed",
        "subtask 1,0 executed",
        "subtask 2,0 executed",
        "subtask 3,0 executed",
        "subtask 1,1 executed",
        "subtask 1,2 executed",
        "subtask 2,1 executed",
        "subtask 2,2 executed",
        "subtask 3,1 executed",
        "subtask 3,2 executed",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
