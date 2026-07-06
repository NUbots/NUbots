/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#include "message/behaviour/Director.hpp"

// Anonymous namespace to avoid name collisions
namespace {

    struct MainTask {};
    struct SubtaskA {};
    struct SubtaskB {};

    /// The number of DirectorState messages that were emitted
    std::atomic<int> state_emissions{0};

    class TestReactor : public TestBase<TestReactor, 1> {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : TestBase(std::move(environment)) {

            // A task that changes multiple provider groups in a single Director cycle
            on<Provide<MainTask>>().then([this] {
                emit<Task>(std::make_unique<SubtaskA>());
                emit<Task>(std::make_unique<SubtaskB>());
            });
            on<Provide<SubtaskA>>().then([] {});
            on<Provide<SubtaskB>>().then([] {});

            // Count every DirectorState that is emitted
            on<Trigger<message::behaviour::DirectorState>>().then([](const message::behaviour::DirectorState&) {  //
                ++state_emissions;
            });

            /**************
             * TEST STEPS *
             **************/
            on<Trigger<Step<1>>, Priority::LOW>().then([this] {  //
                emit<Task>(std::make_unique<MainTask>());
            });
        }
    };
}  // namespace

TEST_CASE("Test that the director state is emitted at most once per director cycle", "[director][state_emission]") {

    NUClear::Configuration config;
    config.default_pool_concurrency = 1;
    NUClear::PowerPlant powerplant(config);
    powerplant.install<module::extension::Director>();
    powerplant.install<TestReactor>();
    powerplant.start();

    // Two director cycles change group state here:
    //   1. the root task pack for MainTask (root group + MainTask group change)
    //   2. MainTask's subtask pack (MainTask, SubtaskA and SubtaskB groups change)
    // Each cycle should emit exactly one DirectorState no matter how many groups changed in it
    INFO("DirectorState was emitted " << state_emissions.load() << " times, expected 2");
    REQUIRE(state_emissions.load() == 2);
}
