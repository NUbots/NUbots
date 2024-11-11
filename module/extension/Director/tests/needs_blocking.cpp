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


    class TestReactor : public TestBase<TestReactor, 3> {
    public:
        struct SimpleTask : Message<SimpleTask> {};
        struct ComplexTask : Message<ComplexTask> {};

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<ComplexTask>, Needs<SimpleTask>>().then(
                [this](const ComplexTask& t) { emit<Task>(std::make_unique<SimpleTask>(t)); });
            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) { finish(t); });

            /**************
             * TEST STEPS *
             **************/
            // Emit a simple task with middling priority (should run)
            on<Trigger<Step<1>>>().then([this] { emit<Task>(std::make_unique<SimpleTask>(), 50); });
            // Emit a complex task with low priority (should be blocked by the needs)
            on<Trigger<Step<2>>>().then([this] { emit<Task>(std::make_unique<ComplexTask>(), 1); });
            // Emit a complex task with high priority (should run)
            on<Trigger<Step<3>>>().then([this] { emit<Task>(std::make_unique<ComplexTask>(), 100); });
        }
    };

}  // namespace

TEST_CASE("Test that a provider will be blocked if its needs aren't met but will run if they are",
          "[director][needs][priority][blocking]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
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
