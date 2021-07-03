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
    struct SimpleMessage {
        explicit SimpleMessage(const int& data_) : data(data_) {}
        int data = 0;
    }

    // The reactor which is taking the place of the module being tested. When testing a real module, the real module
    // would take the place of this reactor
    class TestReactor : public NUClear::Reactor {
    public:
        explicit TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Startup>().then([this] { emit(std::make_unique<SimpleMessage>()); });
            on<Trigger<SimpleMessage>>().then([this](SimpleMessage /*msg*/) {
                emit(std::make_unique<SimpleMessage>(1));
                emit(std::make_unique<SimpleMessage>(2));
            });
            on<Shutdown>().then([this] { shutdown = true; });
        }
    }

}  // namespace

TEST_CASE("Testing the module tester and emission catcher", "[extension][moduletest][ModuleTest]") {

    // We need to test on<Startup> for this reactor (actually for the ModuleTest), so we disable automatic startup
    static constexpr bool STARTUP_REACTOR_AUTOMATICALLY = false;
    auto module_test                                    = ModuleTest<TestReactor>(STARTUP_REACTOR_AUTOMATICALLY);
    SimpleMessage expected_emission_on_startup;
    std::shared_ptr<SimpleMessage> actual_emission_on_startup = std::make_shared<SimpleMessage>();
    // unbinds automatically after reaction
    ModuleTest.bind_catcher_for_next_reaction(actual_emission_on_startup);
    ModuleTest.startup_manually();

    // After we have got the output, then we compare it to what we expected
    // We require that the output is the same as the expected (or within acceptable error), which is trivially true here
}
