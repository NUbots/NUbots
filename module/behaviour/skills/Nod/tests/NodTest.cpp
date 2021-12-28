/*
 * This file is part of NUbots Codebase.
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

#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <nuclear>

#include "Nod.hpp"

#include "extension/Script.hpp"

#include "message/behaviour/Nod.hpp"

#include "utility/behaviour/Action.hpp"
#include "utility/module_test_utils/ModuleTester.hpp"

namespace {

    using utility::behaviour::ActionPriorities;

    std::unique_ptr<ActionPriorities> saved_priority_update = nullptr;

    class TestReactor : public NUClear::Reactor {

    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // On startup we tell the Nod reactor to send ActionPriorities for nodding, by emitting this message
            on<Startup>().then([this]() { emit(std::make_unique<message::behaviour::Nod>()); });

            // If this message is received, the test was successful because the ActionPriorities message was sent
            on<Trigger<ActionPriorities>>().then([this](const ActionPriorities& priority_update) {
                log<NUClear::INFO>("Received a script command message. Test is now over.");
                saved_priority_update = std::make_unique<ActionPriorities>(priority_update);

                // When we receive the ActionPriorities message we can shutdown
                powerplant.shutdown();
            });
        }
    };
}  // namespace

TEST_CASE("Testing the Nod module", "[module][behaviour][skills][Nod]") {

    using module::behaviour::skills::Nod;
    using utility::module_test::ModuleTester;

    static constexpr int NUM_THREADS = 1;

    // Test is for Nod
    ModuleTester<Nod> tester(NUM_THREADS);

    // Install the emission "intercepter" module, which saves the emissions we want to examine
    tester.install<TestReactor>("TestReactor");

    tester.run();

    // Make sure that the action priority update was received, which was the aim of this test
    REQUIRE(saved_priority_update != nullptr);
}
