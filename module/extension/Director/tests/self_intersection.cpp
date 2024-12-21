/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include <fmt/format.h>
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct MainTask {};

    template <int I>
    struct SubTask {
        SubTask() : subtask_id(I) {}
        int subtask_id;
    };

    struct Dependency {
        Dependency(const int& subtask_id_) : subtask_id(subtask_id_) {}
        int subtask_id;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment)
            : TestBase<TestReactor>(std::move(environment)) {

            on<Provide<MainTask>>().then([this] {
                // Emit optional tasks with one blocking the other
                events.push_back("emitting task 1");
                emit<Task>(std::make_unique<SubTask<1>>(), 0, true);
                events.push_back("emitting task 2");
                emit<Task>(std::make_unique<SubTask<2>>(), 0, true);
            });

            on<Provide<SubTask<1>>, Needs<Dependency>>().then([this](const SubTask<1>& t) {
                events.push_back(fmt::format("subtask {} executed", t.subtask_id));

                // Emit the dependency task
                events.push_back(fmt::format("emitting dependency from subtask {}", t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.subtask_id));
            });

            on<Provide<SubTask<2>>, Needs<Dependency>>().then([this](const SubTask<2>& t) {
                events.push_back(fmt::format("subtask {} executed", t.subtask_id));

                // Emit the dependency task
                events.push_back(fmt::format("emitting dependency from subtask {}", t.subtask_id));
                emit<Task>(std::make_unique<Dependency>(t.subtask_id));
            });

            on<Provide<Dependency>>().then([this](const Dependency& t) {
                events.push_back(fmt::format("dependency from subtask {}", t.subtask_id));
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {
                events.push_back("emitting main task");
                emit<Task>(std::make_unique<MainTask>());
            });
            on<Trigger<Step<2>>, Priority::LOW>().then([this] {
                events.push_back("removing main task");
                emit<Task>(std::unique_ptr<MainTask>(nullptr));
            });
            on<Trigger<Step<3>>, Priority::LOW>().then([this] {
                events.push_back("emitting main task");
                emit<Task>(std::make_unique<MainTask>());
            });
            on<Trigger<Step<4>>, Priority::LOW>().then([this] {
                events.push_back("removing main task");
                emit<Task>(std::unique_ptr<MainTask>(nullptr));
            });
            on<Startup>().then([this] {
                emit(std::make_unique<Step<1>>());
                emit(std::make_unique<Step<2>>());
                emit(std::make_unique<Step<3>>());
                emit(std::make_unique<Step<4>>());
            });
        }
    };

}  // namespace

TEST_CASE("Test that when a task has self intersection it applies priority correctly",
          "[director][priority][optional][self_intersection]") {
    // Run the module
    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "emitting main task",
        "emitting task 1",
        "emitting task 2",
        "subtask 1 executed",
        "emitting dependency from subtask 1",
        "dependency from subtask 1",
        "removing main task",
        "emitting main task",
        "emitting task 1",
        "emitting task 2",
        "subtask 1 executed",
        "emitting dependency from subtask 1",
        "dependency from subtask 1",
        "removing main task",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
