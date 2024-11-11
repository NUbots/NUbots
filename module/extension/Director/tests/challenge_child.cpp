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

    class TestReactor : public TestBase<TestReactor, 2> {
    public:
        struct SimpleTask : Message<SimpleTask> {
            using Message::Message;
        };
        struct SubTask : Message<SubTask> {
            using Message::Message;
        };

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<SimpleTask>, Needs<SubTask>>().then([this](const SimpleTask& task) {  //
                emit<Task>(std::make_unique<SubTask>(task));
            });

            on<Provide<SubTask>>().then([this](const SubTask& t) { finish(t); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>>().then([this] { emit<Task>(std::make_unique<SimpleTask>("1")); });
            on<Trigger<Step<2>>>().then([this] { emit<Task>(std::make_unique<SimpleTask>("2")); });
        }
    };
}  // namespace

TEST_CASE("Test that a parent task wins when challenging its child task", "[director][needs][challenge]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    const auto& reactor = powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting task 1",
        "simple task executed",
        "subtask executed",
        "emitting task 2",
        "simple task executed",
        "subtask executed",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, reactor.events));

    // Check the events fired in order and only those events
    REQUIRE(reactor.events == expected);
}
