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

    struct PrimaryTask {};
    struct SecondaryTask {};

    struct SubTask {
        SubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    template <int i>
    struct SimpleTask {
        SimpleTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            // Store the tasks we are given to run
            on<Provide<SimpleTask<1>>>().then([this](const SimpleTask<1>& t) {  //
                events.push_back("simple task 1 from " + t.msg);
            });
            on<Provide<SimpleTask<2>>>().then([this](const SimpleTask<2>& t) {  //
                events.push_back("simple task 2 from " + t.msg);
            });

            on<Provide<PrimaryTask>, SimpleTask<2>, Needs<SubTask>>().then([this]() {
                events.push_back("emitting from primary task");
                emit<Task>(std::make_unique<SimpleTask<2>>("primary task"));
                emit<Task>(std::make_unique<SubTask>("primary task"));
            });

            on<Provide<SubTask>, Needs<SimpleTask<1>>>().then([this](const SubTask& t) {
                events.push_back("emitting from subtask");
                emit<Task>(std::make_unique<SimpleTask<1>>(t.msg));
            });

            on<Provide<SecondaryTask>, Needs<SimpleTask<1>>, Needs<SimpleTask<2>>>().then([this] {
                events.push_back("emitting from secondary task");
                emit<Task>(std::make_unique<SimpleTask<1>>("secondary task"));
                emit<Task>(std::make_unique<SimpleTask<2>>("secondary task"));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>());
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("removing primary task");
                emit<Task>(std::unique_ptr<PrimaryTask>(nullptr));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
            });
        }
    };

}  // namespace

TEST_CASE("Tests a waiting task can take over subtasks of another task that is being removed",
          "[director][needs][watcher][removal]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting primary task",
        "emitting from primary task",
        "simple task 2 from primary task",
        "emitting from subtask",
        "simple task 1 from primary task",
        "emitting secondary task",
        "removing primary task",
        "emitting from secondary task",
        "simple task 1 from secondary task",
        "simple task 2 from secondary task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
