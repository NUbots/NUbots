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

    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };
    struct ComplexTask {
        ComplexTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<ComplexTask>, Needs<SimpleTask>>().then([this](const ComplexTask& task) {
                events.push_back("emitting tasks from complex: " + task.msg);
                emit<Task>(std::make_unique<SimpleTask>(task.msg));
            });
            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) { events.push_back(t.msg); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                // Emit a simple task with middling priority (should run)
                events.push_back("requesting simple task");
                emit<Task>(std::make_unique<SimpleTask>("simple task"), 50);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                // Emit a complex task with low priority (should be blocked by the needs)
                events.push_back("requesting low priority complex task");
                emit<Task>(std::make_unique<ComplexTask>("low priority complex task"), 1);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                // Emit a complex task with high priority (should run)
                events.push_back("requesting high priority complex task");
                emit<Task>(std::make_unique<ComplexTask>("high priority complex task"), 100);
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that a provider will be blocked if its needs aren't met but will run if they are",
          "[director][needs][priority][blocking]") {

    NUClear::PowerPlant::Configuration config;
    config.thread_count = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "requesting simple task",
        "simple task",
        "requesting low priority complex task",
        "requesting high priority complex task",
        "emitting tasks from complex: high priority complex task",
        "high priority complex task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
