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

#include "extension/ModuleTest.hpp"

// Anon namespace so that this reactor is local to this file
namespace {
    // Usually you would use ModuleTest to test an existing reactor with existing messages, but testing ModuleTest
    // itself requires a reactor and a message, so we define them here
    template <int id>
    struct Message {
    }

    // The reactor which is taking the place of the module being tested. When testing a real module, the real module
    // would take the place of this reactor
    class TestReactor : public NUClear::Reactor {
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Trigger<Message<1>>>().then([this](Message<1> /*msg*/) {  //
                emit(std::make_unique<Message<11>>());
            });
            on<Trigger<Message<2>>>().then([this](Message<2> /*msg*/) {
                emit(std::make_unique<Message<21>>());
                emit(std::make_unique<Message<22>>());
            });
        }
    }

}  // namespace

TEST_CASE("Testing the module test", "[extension][ModuleTest]") {
    const Message<0> expected_startup_output{};
    const Message<11> expected_single_output{};
    const auto [expected_double_output_1, expected_double_output_2] = std::tuple<Message<21>, Message<22>>();

    // We need to test on<Startup> for this reactor (actually for the ModuleTest), so we disable automatic startup
    static constexpr bool STARTUP_REACTOR_AUTOMATICALLY = false;
    auto module_test                                    = ModuleTest<TestReactor>(STARTUP_REACTOR_AUTOMATICALLY);

    // TODO(KipHamiltons): make a varargs startup test.

    const Message<11> actual_single_output = module_test.run_reaction<Message<1>, Message<11>>(Message<1>());

    // After we have got the output, then we compare it to what we expected
    // We require that the output is the same as the expected (or within acceptable error), which is trivially true here
    REQUIRE(expected_single_output == actual_single_output);

    const auto [actual_double_output_1, actual_double_output_2] =
        module_test.run_reaction<Message<2>, Message<21>, Message<22>>(Message<2>());

    REQUIRE(expected_double_output_1 == actual_double_output_1);
    REQUIRE(expected_double_output_2 == actual_double_output_2);

    // TODO: call the function and compare the emissions to the expected emissions
}
