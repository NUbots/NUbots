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

    struct MainTask {
        explicit MainTask(int subtask_) : subtask(subtask_) {}
        int subtask;
    };

    template <int I>
    struct Subtask {};

    struct CommonDependency {
        explicit CommonDependency(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<MainTask>>().then([this](const MainTask& task) {
                events.push_back("running main task");
                if (task.subtask == 1) {
                    events.push_back("requesting subtask 1");
                    emit<Task>(std::make_unique<Subtask<1>>());
                }
                else if (task.subtask == 2) {
                    events.push_back("requesting subtask 2");
                    emit<Task>(std::make_unique<Subtask<2>>());
                }
            });

            on<Provide<Subtask<1>>, Needs<CommonDependency>>().then([this] {
                events.push_back("running subtask 1");
                emit<Task>(std::make_unique<CommonDependency>("from subtask 1"));
            });

            on<Provide<Subtask<2>>, Needs<CommonDependency>>().then([this] {
                events.push_back("running subtask 2");
                emit<Task>(std::make_unique<CommonDependency>("from subtask 2"));
            });

            on<Provide<CommonDependency>>().then([this](const CommonDependency& t) {  //
                events.push_back("running common dependency from " + t.msg);
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("requesting main task with subtask 1");
                emit<Task>(std::make_unique<MainTask>(1));
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("requesting main task with subtask 2");
                emit<Task>(std::make_unique<MainTask>(2));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test a provider can replace its task when the new task overlaps in dependencies with the old task",
          "[director][needs][self]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "requesting main task with subtask 1",
        "running main task",
        "requesting subtask 1",
        "running subtask 1",
        "running common dependency from from subtask 1",
        "requesting main task with subtask 2",
        "running main task",
        "requesting subtask 2",
        "running subtask 2",
        "running common dependency from from subtask 2",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
