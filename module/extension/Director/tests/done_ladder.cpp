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

    template <int I, int J>
    struct Level {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 1> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<Level<1, 1>>, Uses<Level<2, 1>>, Uses<Level<2, 2>>>().then(
                [this](const RunReason& run_reason, const Uses<Level<2, 1>>& a, const Uses<Level<2, 2>>& b) {
                    if (run_reason == RunReason::SUBTASK_DONE) {
                        events.push_back("testing Level<1,1> children: a:" + std::to_string(a.done)
                                         + ", b:" + std::to_string(b.done));
                        if (a.done && b.done) {
                            events.push_back("Level<1,1> done");
                            emit<Task>(std::make_unique<Done>());
                        }
                        else {
                            emit<Task>(std::make_unique<Continue>());
                        }
                    }
                    else {
                        // Emit the two level 2 tasks
                        events.push_back("emitting Level<2,{1,2}> tasks");
                        emit<Task>(std::make_unique<Level<2, 1>>());
                        emit<Task>(std::make_unique<Level<2, 2>>());
                    }
                });

            // Provides for the level 2 tasks which emit two level 3 tasks each
            on<Provide<Level<2, 1>>, Needs<Level<3, 1>>, Needs<Level<3, 2>>>().then(
                [this](const RunReason& run_reason, const Uses<Level<3, 1>>& a, const Uses<Level<3, 2>>& b) {
                    if (run_reason == RunReason::SUBTASK_DONE) {
                        events.push_back("testing Level<2,1> children: a:" + std::to_string(a.done)
                                         + ", b:" + std::to_string(b.done));
                        if (a.done && b.done) {
                            events.push_back("Level<2,1> done");
                            emit<Task>(std::make_unique<Done>());
                        }
                        else {
                            emit<Task>(std::make_unique<Continue>());
                        }
                    }
                    else {
                        // Emit the two level 3 tasks
                        events.push_back("emitting Level<3,{1,2}> tasks");
                        emit<Task>(std::make_unique<Level<3, 1>>());
                        emit<Task>(std::make_unique<Level<3, 2>>());
                    }
                });

            on<Provide<Level<2, 2>>, Needs<Level<3, 3>>, Needs<Level<3, 4>>>().then(
                [this](const RunReason& run_reason, const Uses<Level<3, 3>>& a, const Uses<Level<3, 4>>& b) {
                    if (run_reason == RunReason::SUBTASK_DONE) {
                        events.push_back("testing Level<2,2> children: a:" + std::to_string(a.done)
                                         + ", b:" + std::to_string(b.done));
                        if (a.done && b.done) {
                            events.push_back("Level<2,2> done");
                            emit<Task>(std::make_unique<Done>());
                        }
                        else {
                            emit<Task>(std::make_unique<Continue>());
                        }
                    }
                    else {
                        // Emit the two level 3 tasks
                        events.push_back("emitting Level<3,{3,4}> tasks");
                        emit<Task>(std::make_unique<Level<3, 3>>());
                        emit<Task>(std::make_unique<Level<3, 4>>());
                    }
                });

            // Provides for level 3 tasks
            on<Provide<Level<3, 1>>>().then([this] {
                events.push_back("Level<3,1> done");
                emit<Task>(std::make_unique<Done>());
            });
            on<Provide<Level<3, 2>>>().then([this] {
                events.push_back("Level<3,2> done");
                emit<Task>(std::make_unique<Done>());
            });
            on<Provide<Level<3, 3>>>().then([this] {
                events.push_back("Level<3,3> done");
                emit<Task>(std::make_unique<Done>());
            });
            on<Provide<Level<3, 4>>>().then([this] {
                events.push_back("Level<3,4> done");
                emit<Task>(std::make_unique<Done>());
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>>().then([this] {
                events.push_back("emitting initial task");
                emit<Task>(std::make_unique<Level<1, 1>>());
            });
        }

        bool executed = false;
    };
}  // namespace

TEST_CASE("Test that a ladder of done tasks can be used to produce an aggregate done", "[director][done]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting initial task",
        "emitting Level<2,{1,2}> tasks",
        "emitting Level<3,{1,2}> tasks",
        "emitting Level<3,{3,4}> tasks",
        "Level<3,1> done",
        "Level<3,2> done",
        "Level<3,3> done",
        "Level<3,4> done",
        "testing Level<2,1> children: a:1, b:0",
        "testing Level<2,1> children: a:1, b:1",
        "Level<2,1> done",
        "testing Level<2,2> children: a:1, b:0",
        "testing Level<2,2> children: a:1, b:1",
        "Level<2,2> done",
        "testing Level<1,1> children: a:1, b:0",
        "testing Level<1,1> children: a:1, b:1",
        "Level<1,1> done",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
