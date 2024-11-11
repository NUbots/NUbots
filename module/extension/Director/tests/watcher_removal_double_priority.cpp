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
#include <nuclear>

#include "Director.hpp"
#include "TestBase.hpp"
#include "util/diff_string.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct PrimaryTask {};
    struct SecondaryTask {};
    struct TertiaryTask {};

    struct SubTask {
        SubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    struct SubSubTask {
        SubSubTask(const std::string& msg_) : msg(msg_) {}
        std::string msg;
    };

    std::vector<std::string> events;

    class TestReactor : public TestBase<TestReactor, 4> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<PrimaryTask>>().then([this] { emit<Task>(std::make_unique<SubTask>("primary task")); });

            on<Provide<SubTask>, Needs<SubSubTask>>().then(
                [this](const SubTask& t) { emit<Task>(std::make_unique<SubSubTask>(t.msg)); });

            on<Provide<SubSubTask>>().then([this](const SubSubTask& t) { events.push_back(t.msg); });

            on<Provide<SecondaryTask>, Needs<SubTask>>().then(
                [this] { emit<Task>(std::make_unique<SubTask>("secondary task")); });

            on<Provide<TertiaryTask>, Needs<SubTask>>().then(
                [this] { emit<Task>(std::make_unique<SubTask>("tertiary task")); });

            /**************
             * TEST STEPS *
             **************/
            // PrimaryTask takes ahold of SubTask and SubSubTask
            on<Trigger<Step<1>>>().then([this] {
                events.push_back("emitting primary task");
                emit<Task>(std::make_unique<PrimaryTask>());
            });

            // SecondaryTask needs SubTask, so it will watch
            on<Trigger<Step<2>>>().then([this] {
                events.push_back("emitting secondary task");
                emit<Task>(std::make_unique<SecondaryTask>());
            });

            // TertiaryTask needs SubTask, so it will watch
            on<Trigger<Step<3>>>().then([this] {
                events.push_back("emitting tertiary task");
                emit<Task>(std::make_unique<TertiaryTask>());
            });

            // Remove PrimaryTask so SecondaryTask can take over
            on<Trigger<Step<4>>>().then([this] {
                events.push_back("removing primary task");
                emit<Task>(std::unique_ptr<PrimaryTask>(nullptr));
            });
        }
    };

}  // namespace

TEST_CASE(
    "Test that a watcher can take over from another provider while there is another watcher and no priority is set",
    "[director][remove][watcher][double][priority]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {"emitting primary task",
                                         "primary task",
                                         "emitting secondary task",
                                         "emitting tertiary task",
                                         "removing primary task",
                                         "secondary task"};

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
