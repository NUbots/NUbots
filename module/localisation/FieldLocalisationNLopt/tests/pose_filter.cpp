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
#include <cmath>

#include "localisation/PoseFilter.hpp"

using module::localisation::PoseFilter;

TEST_CASE("PoseFilter converges to a constant measurement", "[PoseFilter]") {
    PoseFilter filter;
    filter.reset(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0));

    const Eigen::Vector3d z(1.0, 2.0, 0.5);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;

    double previous_trace = filter.cov.trace();
    for (int i = 0; i < 50; ++i) {
        filter.update(z, R);
        // Covariance should shrink (or stay the same) with every update towards a consistent measurement.
        REQUIRE(filter.cov.trace() <= previous_trace + 1e-9);
        previous_trace = filter.cov.trace();
    }

    REQUIRE(std::abs(filter.mean.x() - z.x()) < 1e-3);
    REQUIRE(std::abs(filter.mean.y() - z.y()) < 1e-3);
    REQUIRE(std::abs(filter.mean.z() - z.z()) < 1e-3);
}

TEST_CASE("PoseFilter update wraps the innovation across +-pi", "[PoseFilter]") {
    PoseFilter filter;
    // Mean sits just below +pi; measurement sits just above -pi. The true angular distance is small, but a
    // naive (non-wrapped) subtraction would see a jump of almost 2*pi.
    filter.reset(Eigen::Vector3d(0.0, 0.0, M_PI - 0.05), Eigen::Vector3d(0.1, 0.1, 0.1));

    const Eigen::Vector3d z(0.0, 0.0, -M_PI + 0.05);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;

    filter.update(z, R);

    // The updated mean should have moved only a small (wrapped) distance from its prior, not nearly 2*pi.
    double raw_diff = std::abs(filter.mean.z() - (M_PI - 0.05));
    REQUIRE(raw_diff < 0.2);

    // Mean should stay within (-pi, pi].
    REQUIRE(filter.mean.z() <= M_PI);
    REQUIRE(filter.mean.z() > -M_PI);
}

TEST_CASE("PoseFilter predict grows covariance proportionally to motion", "[PoseFilter]") {
    PoseFilter filter;
    filter.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, 0.1, 0.1));

    const Eigen::Vector3d rate(0.01, 0.02, 0.005);
    const Eigen::Vector3d initial_diagonal = filter.cov.diagonal();

    filter.predict(1.0, rate);
    Eigen::Vector3d after_unit_motion = filter.cov.diagonal();
    REQUIRE((after_unit_motion - initial_diagonal - rate).norm() < 1e-9);

    filter.predict(2.0, rate);
    Eigen::Vector3d after_more_motion = filter.cov.diagonal();
    REQUIRE((after_more_motion - after_unit_motion - 2.0 * rate).norm() < 1e-9);

    // More motion should always grow (never shrink) the covariance.
    REQUIRE(after_more_motion.x() > after_unit_motion.x());
}

TEST_CASE("PoseFilter update contracts covariance given a finite measurement covariance", "[PoseFilter]") {
    PoseFilter filter;
    filter.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 1.0, 1.0));

    const double trace_before = filter.cov.trace();
    filter.update(Eigen::Vector3d(0.1, -0.2, 0.05), Eigen::Matrix3d::Identity() * 0.1);
    const double trace_after = filter.cov.trace();

    REQUIRE(trace_after < trace_before);
}
