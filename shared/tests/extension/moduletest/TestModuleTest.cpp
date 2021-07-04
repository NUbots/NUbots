/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include <catch.hpp>
#include <nuclear>
#include <utility>

#include "extension/ModuleTest/ModuleTest.hpp"

// Anon namespace so that this reactor is local to this file
namespace {

    // We shouldn't emit anything on shutdown, so we'll use this to track whether the shutdown went to plan
    // This serves as a proxy for testing real on<Shutdown> effects
    static bool is_shutdown = false;

    // Usually you would use ModuleTest to test an existing reactor with existing messages, but testing ModuleTest
    // itself requires a reactor and a message, so we define them here
    template <int id>
    struct SimpleMessage {
        SimpleMessage() = default;
        explicit SimpleMessage(const int& data_) : data(data_) {}
        int data = 0;
    };

    // The reactor which is taking the place of the module being tested. When testing a real module, the real module
    // would take the place of this reactor
    class TestReactor : public NUClear::Reactor {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Startup>().then([this] {
                is_shutdown = false;
                emit(std::make_unique<SimpleMessage<0>>(42));
            });
            on<Trigger<SimpleMessage<1>>>().then([this](SimpleMessage<1> /*msg*/) {
                emit(std::make_unique<SimpleMessage<2>>(42));
                emit(std::make_unique<SimpleMessage<3>>(31415));
            });
            on<Shutdown>().then([this] { is_shutdown = true; });
        }
    };

}  // namespace

TEST_CASE("Testing the module tester on the Startup of the TestReactor", "[extension][moduletest][ModuleTest]") {

    auto module_test = extension::moduletest::ModuleTest<TestReactor>();
    // Our ground truth/expected message to be emitted when we start up. We'll compare the actual result to this one
    const auto expected_emission_on_startup = SimpleMessage<0>(42);
    // This can be thought of as an out-parameter of a manually triggered reaction. We give ModuleTest a pointer
    // to an instance of the message type(s) we expect to be emitted, to catch the emitted message
    auto actual_emission_on_startup = std::make_shared<SimpleMessage<0>>();
    // We pass the pointer to the ModuleTest so that it knows where to write it
    // The pointer and its associated handle unbinds automatically after reaction
    module_test.bind_catcher_for_next_reaction(actual_emission_on_startup);
    // Start up manually, to test our on<Startup>. This binds actual_emission_on_startup to the emitted message
    module_test.start_test();
    // After we have got the output, we compare it to what we expected
    // We require that the output is the same as the expected (or within acceptable error)
    REQUIRE(expected_emission_on_startup.data == (*actual_emission_on_startup).data);
}

TEST_CASE("Testing the module tester on generic reactions of the TestReactor", "[extension][moduletest][ModuleTest]") {

    // We're not testing on<Startup>, so we can start the reactor automatically
    auto module_test = extension::moduletest::ModuleTest<TestReactor>();
    // Ground truth/expected emissions, to compare to with the actual result
    const auto expected_first_emission  = SimpleMessage<2>(42);
    const auto expected_second_emission = SimpleMessage<3>(31415);

    // Set up the actual emissions variables to bind the results to
    // Note that we want them to have different values for their data to the ground truth/expected results, so that
    // if there wasn't an emission of these, the test doesn't pass
    auto actual_first_emission = std::make_shared<SimpleMessage<2>>();
    module_test.bind_catcher_for_next_reaction(actual_first_emission);
    auto actual_second_emission = std::make_shared<SimpleMessage<3>>();
    module_test.bind_catcher_for_next_reaction(actual_second_emission);

    // Trigger the reaction manually
    module_test.emit<SimpleMessage<1>>(std::make_unique<SimpleMessage<1>>());

    // Compare actual results to expected results
    REQUIRE(expected_first_emission.data == actual_first_emission->data);
    REQUIRE(expected_second_emission.data == actual_second_emission->data);
}

TEST_CASE("Testing the module tester on the Shutdown of the TestReactor", "[extension][moduletest][ModuleTest]") {

    // auto module_test = extension::moduletest::ModuleTest<TestReactor>();
    // // Define our ground truth/expected result
    // static constexpr bool HAS_SHUTDOWN = true;
    // // Set up our actual result variable
    // is_shutdown = false;
    // module_test.shutdown_manually();
    // // Compare ground truth/expected result with actual result
    // REQUIRE(HAS_SHUTDOWN == is_shutdown);
}
