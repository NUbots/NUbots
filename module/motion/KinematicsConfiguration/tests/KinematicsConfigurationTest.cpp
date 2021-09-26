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

#include "utility/strutil/ansi.hpp"
#include "utility/support/ModuleTester.hpp"

namespace module::motion::kinematicsconfigurationtest {
    using message::motion::KinematicsModel;

    std::unique_ptr<KinematicsModel> saved_model = nullptr;

    class TestReactor : public NUClear::Reactor {
    public:
        TestReactor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Trigger on the KinematicsModel message
            on<Trigger<KinematicsModel>>().then([this](const KinematicsModel& model) {
                log<NUClear::INFO>("Received a KinematicsModel message. Test is now over.");
                saved_model = std::make_unique<KinematicsModel>(model);

                // When we receive the KinematicsModel message we can shutdown
                powerplant.shutdown();
            });
        }
    };
}  // namespace module::motion::kinematicsconfigurationtest

TEST_CASE("Testing the Kinematics Configuration module", "[module][motion][KinematicsConfiguration]") {

    using module::motion::kinematicsconfigurationtest::saved_model;
    using utility::support::ModuleTester;

    static constexpr int NUM_THREADS = 2;

    ModuleTester<module::motion::KinematicsConfiguration> tester(NUM_THREADS);
    tester.install<module::extension::FileWatcher>("FileWatcher");
    tester.install<module::motion::kinematicsconfigurationtest::TestReactor>("TestReactor");
    tester.run();

    // Require that a model was saved
    REQUIRE(saved_model != nullptr);

    // Now check values in model to ensure correctness
    REQUIRE(saved_model->head.INTERPUPILLARY_DISTANCE == 0.068f);
}
