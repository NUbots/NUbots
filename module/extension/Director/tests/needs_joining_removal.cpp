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
    template <int i>
    struct ComplexTask {};
    struct AnotherComplexTask {};
    struct VeryComplexTask {};

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            // Store the tasks we are given to run
            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) {  //
                events.push_back("simple task - " + t.msg);
            });

            on<Provide<ComplexTask<1>>, Needs<SimpleTask>>().then([this] {
                events.push_back("emitting tasks from complex task 1");
                emit<Task>(std::make_unique<SimpleTask>("complex task 1"));
            });

            on<Provide<ComplexTask<2>>, Needs<SimpleTask>>().then([this] {
                events.push_back("emitting tasks from complex task 2");
                emit<Task>(std::make_unique<SimpleTask>("complex task 2"));
            });

            on<Provide<VeryComplexTask>, Needs<ComplexTask<2>>>().then([this] {
                events.push_back("emitting tasks from very complex task");
                emit<Task>(std::make_unique<ComplexTask<2>>());
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting complex task 1");
                emit<Task>(std::make_unique<ComplexTask<1>>(), 50);
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting very complex task");
                emit<Task>(std::make_unique<VeryComplexTask>(), 40);
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing complex task 1");
                emit<Task>(std::unique_ptr<ComplexTask<1>>(nullptr));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that when the needs a higher task is blocked on are removed, the higher task will run",
          "[director][needs][joining][removal]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting complex task 1",
        "emitting tasks from complex task 1",
        "simple task - complex task 1",
        "emitting very complex task",
        "removing complex task 1",
        "emitting tasks from very complex task",
        "emitting tasks from complex task 2",
        "simple task - complex task 2",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
