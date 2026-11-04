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
#include "DynamicInverse.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

using Catch::Approx;
using module::planning::dynamic_inverted_walk_mpc::DynamicInverse;
using utility::planning::HammersteinModel;

namespace {

    /// @brief The K1's model in the module's configuration (tests run in the build directory, where it is copied)
    HammersteinModel k1_model() {
        return HammersteinModel::from_yaml(YAML::LoadFile("config/DynamicInvertedMPCWalkPathModel.yaml"));
    }

    const Eigen::Vector3d RANGE(1.75, 1.75, 1.5);

    /// @brief The inverse as the module builds it: every 0.01 out to the identified range, one planner step (0.1 s)
    /// ahead
    DynamicInverse k1_inverse() {
        return DynamicInverse(k1_model(), RANGE, 0.01, 0.1);
    }

    /// @brief How close the inverse's command delivers what was asked, by the model: the tie tolerance, and the
    /// refinement's step on the steepest edge
    constexpr double MATCH = 0.01;

    /// @brief Settles the estimate on the command, as though the robot had walked with it for a while
    void settle(DynamicInverse& inverse, const Eigen::Vector3d& command) {
        inverse.record(command);
        inverse.advance(10.0);
    }

}  // namespace

TEST_CASE("The estimate runs the model from the commands recorded", "[DynamicInverse]") {
    const HammersteinModel model = k1_model();
    DynamicInverse inverse       = k1_inverse();
    CHECK(inverse.samples() == 5);

    const Eigen::Vector3d u(0.6, -0.5, 0.8);
    inverse.record(u);
    // Advances shorter than a sample carry their remainder: 0.03 + 0.07 s is five samples
    inverse.advance(0.03);
    inverse.advance(0.07);
    const HammersteinModel::Lags expected = model.step(HammersteinModel::Lags::Zero(), u, 5);
    CHECK((inverse.delivered() - HammersteinModel::delivered(expected)).norm() < 1e-12);
    CHECK((inverse.after_step(u) - HammersteinModel::delivered(model.step(expected, u, 5))).norm() < 1e-12);

    inverse.reset();
    CHECK(inverse.delivered().isZero());
}

TEST_CASE("While walking, the command delivers the velocity asked for one step later", "[DynamicInverse]") {
    // Turning while walking stays below |ω·vx| = 0.84 of command, beyond which the model's product path turns less for
    // a larger command: there, speeding up loses turn, and the translation-first search can't trade one for the other
    DynamicInverse inverse       = k1_inverse();
    const HammersteinModel model = k1_model();
    for (const Eigen::Vector3d& walking : {Eigen::Vector3d(0.8, 0.0, 0.0),
                                           Eigen::Vector3d(0.5, 0.6, 0.0),
                                           Eigen::Vector3d(-0.6, 0.0, 0.5),
                                           Eigen::Vector3d(0.8, 0.0, -0.5)}) {
        settle(inverse, walking);
        const Eigen::Vector3d now = model.steady_state(walking);
        for (const Eigen::Vector3d& change : {Eigen::Vector3d(0.05, 0.0, 0.0),
                                              Eigen::Vector3d(-0.05, 0.05, 0.0),
                                              Eigen::Vector3d(0.0, -0.05, 0.1),
                                              Eigen::Vector3d(0.0, 0.0, 0.0)}) {
            const Eigen::Vector3d wanted    = now + change;
            const Eigen::Vector3d command   = inverse.inverse(wanted);
            const Eigen::Vector3d delivered = inverse.after_step(command);
            INFO("walking " << walking.transpose() << " wants " << wanted.transpose() << ": command "
                            << command.transpose() << " delivers " << delivered.transpose());
            CHECK((delivered.head<2>() - wanted.head<2>()).norm() <= MATCH);
            CHECK(std::abs(delivered.z() - wanted.z()) <= 2 * MATCH);
        }
        inverse.reset();
    }
}

TEST_CASE("Speeding up commands past the steady state, to beat the lag", "[DynamicInverse]") {
    // vx's lag only covers about a quarter of a change in 0.1 s, so the command leads: from 0.5 m/s to 0.6 m/s takes
    // a command well past the 0.6 m/s that holds it there
    DynamicInverse inverse       = k1_inverse();
    const HammersteinModel model = k1_model();
    settle(inverse, {0.55, 0.0, 0.0});
    const double now    = inverse.delivered().x();
    const double wanted = now + 0.1;
    const Eigen::Vector3d command = inverse.inverse({wanted, inverse.delivered().y(), inverse.delivered().z()});
    CHECK(inverse.after_step(command).x() == Approx(wanted).margin(MATCH));
    CHECK(model.steady_state(command).x() > wanted + 0.2);
}

TEST_CASE("A change the lag can't make in one step gets the command at the edge of the range", "[DynamicInverse]") {
    // Or the smallest within the tie tolerance of it, as the model's response flattens towards the edge
    DynamicInverse inverse        = k1_inverse();
    const Eigen::Vector3d command = inverse.inverse({1.2, 0.0, 0.0});
    CHECK(command.x() > RANGE.x() - 0.05);
    CHECK(command.x() <= RANGE.x());
    CHECK(inverse.after_step(command).x() < 1.2);
}

TEST_CASE("From standing, a small sidestep is commanded along the edge of the dead zone", "[DynamicInverse]") {
    // The model's vy dead zone ends in an edge where vy rises 15 times as fast as its command, between 0.42 and 0.45
    // m/s of command; a grid step alone can't land on the velocities along it
    DynamicInverse inverse = k1_inverse();
    for (const double vy : {0.02, 0.05, 0.1}) {
        const Eigen::Vector3d command = inverse.inverse({0.0, vy, 0.0});
        INFO("vy " << vy << " gets " << command.transpose());
        CHECK(inverse.after_step(command).y() == Approx(vy).margin(MATCH));
    }
}

TEST_CASE("Standing still holds the robot with commands inside the dead zones", "[DynamicInverse]") {
    // The model drifts at the zero command, which the inverse cancels
    DynamicInverse inverse        = k1_inverse();
    const Eigen::Vector3d command = inverse.inverse(Eigen::Vector3d::Zero());
    CHECK(command.cwiseAbs().maxCoeff() < 0.1);
    CHECK(inverse.after_step(command).cwiseAbs().maxCoeff() < MATCH);
}

TEST_CASE("The planner's limits are what the model settles to within the range", "[DynamicInverse]") {
    const HammersteinModel model = k1_model();
    const DynamicInverse inverse = k1_inverse();
    CHECK(inverse.upper().x() == Approx(model.steady_state({1.75, 0.0, 0.0}).x()));
    CHECK(inverse.lower().y() <= model.steady_state({0.0, -1.75, 0.0}).y() + 1e-12);
    CHECK(inverse.upper().z() >= model.steady_state({0.0, 0.0, 1.5}).z() - 1e-12);
    CHECK_THROWS_AS(DynamicInverse(model, RANGE, 0.0, 0.1), std::runtime_error);
}

