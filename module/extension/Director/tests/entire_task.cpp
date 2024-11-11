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

    template <int i>
    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct ComplexTask {
        ComplexTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct BlockerTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 3> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<SimpleTask<0>>>().then([this](const SimpleTask<0>& t) { events.push_back(t.msg); });
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) { events.push_back(t.msg); });

            on<Provide<ComplexTask>>().then([this](const ComplexTask& t) {
                emit<Task>(std::make_unique<SimpleTask<0>>(t));
                emit<Task>(std::make_unique<SimpleTask<1>>(t));
            });

            on<Provide<BlockerTask>>().then(
                [this](const BlockerTask& t) { emit<Task>(std::make_unique<SimpleTask<0>>(t)); });

            /**************
             * TEST STEPS *
             **************/
            // Emit a blocker task that will use SimpleTask<0> to block the complex task
            on<Trigger<Step<1>>>().then([this] { emit<Task>(std::make_unique<BlockerTask>(), 50); });
            // Emit the complex task that should be blocked by the blocker task
            on<Trigger<Step<2>>>().then([this] { emit<Task>(std::make_unique<ComplexTask>("low priority"), 10); });
            // Emit another complex task that should have high enough priority to execute over the blocker
            on<Trigger<Step<3>>>().then([this] { emit<Task>(std::make_unique<ComplexTask>("high priority"), 100); });
        }
    };

}  // namespace

TEST_CASE("Test that if all the non optional tasks can't be executed none of them will be",
          "[director][priority][entire]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting blocker task",
        "emitting blocker simple task",
        "from blocker",
        "emitting low priority complex task",
        "emitting tasks from complex low priority",
        "emitting high priority complex task",
        "emitting tasks from complex high priority",
        "from complex high priority",
        "from complex high priority",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
