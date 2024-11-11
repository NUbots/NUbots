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

    class TestReactor : public TestBase<TestReactor, 4> {
    public:
        template <int id>
        struct Runner : Message<Runner<id>> {};
        struct SimpleTask : Message<SimpleTask> {};

        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            on<Provide<SimpleTask>>().then([this](const SimpleTask& t) { finish(t); });
            on<Provide<Runner<1>>>().then([this] { emit<Task>(std::make_unique<SimpleTask>("a")); });
            on<Provide<Runner<2>>>().then([this] { emit<Task>(std::make_unique<SimpleTask>("b")); });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>>().then([this] { log_emit<Task>(std::make_unique<Runner<1>>(), 10); });
            on<Trigger<Step<2>>>().then([this] { log_emit<Task>(std::make_unique<Runner<2>>(), 10); });
            on<Trigger<Step<3>>>().then([this] { log_emit<Task>(std::make_unique<Runner<2>>(), 20); });
            on<Trigger<Step<4>>>().then([this] { log_emit<Task>(std::make_unique<Runner<1>>(), 20); });
        }
    };
}  // namespace

TEST_CASE("Test when two tasks of equal priority are emitted the one that was emitted first wins",
          "[director][priority][equal]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    std::vector<std::string> expected = {
        "starting a",
        "runner a",
        "simple task from a",
        "starting b",
        "runner b",
        "upping b priority",
        "runner b",
        "simple task from b",
        "upping a priority",
        "runner a",
    };

    // Make an info print the diff in an easy to read way if we fail
    INFO(util::diff_string(expected, events));

    // Check the events fired in order and only those events
    REQUIRE(events == expected);
}
