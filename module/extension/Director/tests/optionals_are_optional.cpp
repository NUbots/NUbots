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
    struct BlockerTask {
        BlockerTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<SimpleTask<0>>>().then([this](const SimpleTask<0>& t) {  //
                events.push_back("task 0 " + t.msg);
            });
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) {  //
                events.push_back("task 1 " + t.msg);
            });

            on<Provide<ComplexTask>>().then([this](const ComplexTask& t) {
                events.push_back("emitting tasks from complex " + t.msg);
                // One required, one optional
                emit<Task>(std::make_unique<SimpleTask<0>>("from complex " + t.msg), 1, true);
                emit<Task>(std::make_unique<SimpleTask<1>>("from complex " + t.msg), 1, false);
            });

            on<Provide<BlockerTask>>().then([this](const BlockerTask& t) {  //
                events.push_back("emitting " + t.msg + " blocker simple task");
                emit<Task>(std::make_unique<SimpleTask<0>>("from blocker " + t.msg));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit an initial complex task which should run
                events.push_back("emitting initial complex task");
                emit<Task>(std::make_unique<ComplexTask>("initial task"), 10);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a blocker task that will use SimpleTask<0> to block the optional part of the complex task
                events.push_back("emitting required blocker task");
                emit<Task>(std::make_unique<BlockerTask>("required"), 50);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit an updated complex task that should be blocked by the blocker task
                // However the non optional part should still run
                events.push_back("emitting updated complex task");
                emit<Task>(std::make_unique<ComplexTask>("updated task"), 10);
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                // Emit another complex task that should have high enough priority to execute over the blocker
                // except that since it's optional it won't be able to
                events.push_back("emitting high priority complex task");
                emit<Task>(std::make_unique<ComplexTask>("high priority"), 100);
            });
            on<Trigger<Step<5>>, Priority::LOW>().then([this] {
                // Emit an optional blocker task which the complex task should override
                events.push_back("emitting optional blocker task");
                emit<Task>(std::make_unique<BlockerTask>("optional"), 50, true);
            });

            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
                emit(std::make_unique<Step<5>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that if a task is optional then it does not need to be executed for the other tasks to run",
          "[director][priority][optional]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting initial complex task",
        "emitting tasks from complex initial task",
        "task 1 from complex initial task",
        "task 0 from complex initial task",
        "emitting required blocker task",
        "emitting required blocker simple task",
        "task 0 from blocker required",
        "emitting updated complex task",
        "emitting tasks from complex updated task",
        "task 1 from complex updated task",
        "emitting high priority complex task",
        "emitting tasks from complex high priority",
        "task 1 from complex high priority",
        "emitting optional blocker task",
        "task 0 from complex high priority",
        "emitting optional blocker simple task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
