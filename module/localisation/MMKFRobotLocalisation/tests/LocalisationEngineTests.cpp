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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <catch.hpp>

#include <nuclear>
#include "utility/math/angle.h"
#include "utility/localisation/transform.h"
#include "MultiModalRobotModel.h"

using utility::localisation::transform::WorldToRobotTransform;
using utility::localisation::transform::RobotToWorldTransform;
using utility::math::angle::normalizeAngle;
using module::localisation::MultiModalRobotModel;
using module::localisation::RobotHypothesis;
using utility::math::kalman::UKF;
using module::localisation::robot::RobotModel;

TEST_CASE("Angle convinience functions should handle corner cases", "[math][angle]") {

    // INFO("Testing the FSR centre conversions");

    for(int i = 0; i < 10000; ++i) {
        float a = i * M_PI / 10000.0;

        REQUIRE(normalizeAngle(a + 2 * M_PI) ==
            normalizeAngle(a));

        REQUIRE(normalizeAngle(a - 2 * M_PI) ==
            normalizeAngle(a));
    }

    REQUIRE(normalizeAngle(0.0) == Approx(0.0));
    REQUIRE(normalizeAngle( M_PI) == Approx(M_PI));
    REQUIRE(normalizeAngle(-M_PI) == Approx(M_PI));
    REQUIRE(normalizeAngle( M_PI * 2) == Approx(0.0));
    REQUIRE(normalizeAngle(-M_PI * 2) == Approx(0.0));
    REQUIRE(normalizeAngle( M_PI * 2 - 0.1) == Approx(-0.1));
    REQUIRE(normalizeAngle(-M_PI * 2 + 0.1) == Approx( 0.1));
    REQUIRE(normalizeAngle( M_PI * 77 - 0.1) == Approx(M_PI - 0.1));
    REQUIRE(normalizeAngle(-M_PI * 77 + 0.1) == Approx(-M_PI + 0.1));
}

TEST_CASE("RobotToWorldTransform should be inverse of WorldToRobotTransform") {

    // INFO("Testing the FSR centre conversions");

    arma::vec2 robot_pos = { 3, 2 };
    arma::vec2 robot_heading = arma::normalise(arma::vec({ 1, 5 }));
    arma::vec2 field_ball = { -3, -1 };

    auto robot_ball = WorldToRobotTransform(robot_pos, robot_heading, field_ball);
    auto result_ball = RobotToWorldTransform(robot_pos, robot_heading, robot_ball);

    REQUIRE(field_ball(0) == Approx(result_ball(0)));
    REQUIRE(field_ball(1) == Approx(result_ball(1)));
}

TEST_CASE("MultiModalRobotModel::MergeSimilarModels tests") {
    {
        INFO("Test merge with single input model");
        MultiModalRobotModel mmrm;
        REQUIRE(mmrm.robot_models_.size() == 1);
        mmrm.MergeSimilarModels();
        REQUIRE(mmrm.robot_models_.size() == 1);
    }

    {
        INFO("Test merge with two identical input models");
        MultiModalRobotModel mmrm;
        std::cout << *mmrm.robot_models_.back() << std::endl;
        mmrm.robot_models_.push_back(std::make_unique<RobotHypothesis>());
        std::cout << *mmrm.robot_models_.back() << std::endl;
        REQUIRE(mmrm.robot_models_.size() == 2);
        mmrm.MergeSimilarModels();
        REQUIRE(mmrm.robot_models_.size() == 1);
    }

    {
        INFO("Test merge with three identical input models");
        MultiModalRobotModel mmrm;
        mmrm.robot_models_.push_back(std::make_unique<RobotHypothesis>());
        mmrm.robot_models_.push_back(std::make_unique<RobotHypothesis>());
        REQUIRE(mmrm.robot_models_.size() == 3);
        mmrm.MergeSimilarModels();
        REQUIRE(mmrm.robot_models_.size() == 1);
    }

    {
        INFO("Test merge with two identical and one different input models");
        MultiModalRobotModel mmrm;
        auto hyp = std::make_unique<RobotHypothesis>();
        hyp->filter_ = UKF<RobotModel>(arma::vec({10, 20, 30}));
        mmrm.robot_models_.push_back(std::move(hyp));
        mmrm.robot_models_.push_back(std::make_unique<RobotHypothesis>());
        REQUIRE(mmrm.robot_models_.size() == 3);
        mmrm.MergeSimilarModels();
        REQUIRE(mmrm.robot_models_.size() == 2);
    }
}
