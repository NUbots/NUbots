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
#include <memory>
#include <nuclear>

#include "KinematicsConfiguration.hpp"

#include "extension/FileWatcher/src/FileWatcher.hpp"

#include "message/motion/KinematicsModel.hpp"

#include "utility/module_test_utils/ModuleTester.hpp"
#include "utility/strutil/ansi.hpp"

namespace {
    using message::motion::KinematicsModel;

    KinematicsModel saved_model{};

    class TestReactor : public NUClear::Reactor {
    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Trigger on the KinematicsModel message
            on<Trigger<KinematicsModel>>().then([this](const KinematicsModel& model) {
                log<NUClear::INFO>("Received a KinematicsModel message. Test is now over.");
                saved_model = model;

                // When we receive the KinematicsModel message we can shutdown
                powerplant.shutdown();
            });
        }
    };
}  // namespace

TEST_CASE("Testing the Kinematics Configuration module", "[module][motion][KinematicsConfiguration]") {

    using module::extension::FileWatcher;
    using module::motion::KinematicsConfiguration;
    using utility::module_test::ModuleTester;

    static constexpr int NUM_THREADS = 2;

    // Test is for KinematicsConfiguration
    ModuleTester<KinematicsConfiguration> tester(NUM_THREADS);

    // Configuration tasks depend on FileWatcher
    tester.install<FileWatcher>("FileWatcher");

    // Finally, we install the emission "intercepter" module, which saves the emissions for us
    tester.install<TestReactor>("TestReactor");

    tester.run();

    std::cout << "Test";

    // We needed to have saved a model. If we didn't, we failed the test
    // if (saved_model != nullptr) {
    // TODO(Devops&QA/Motion): Make this test actually check the config values as found in the file, rather than
    //                         hardcoding this value which is subject to change
    // Now check values in model to ensure correctness
    REQUIRE(saved_model.head.INTERPUPILLARY_DISTANCE == 0.068f);
    // }
    // else {
    //     FAIL("No model was saved.");
    // }
}
