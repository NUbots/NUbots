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
#include "Track.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <stdexcept>

using Catch::Approx;
using module::tools::walk_track::Segment;
using module::tools::walk_track::Track;

namespace {

    /// @brief A segment with only the given fields set
    Segment segment(const double speed,
                    const double duration,
                    const double acceleration = 0.0,
                    const double curvature    = 0.0,
                    const double turn         = 0.0) {
        return Segment{speed, acceleration, curvature, turn, duration};
    }

}  // namespace

TEST_CASE("A straight segment moves at its speed along its direction", "[WalkTrackBenchmark][Track]") {
    const Track track(Eigen::Vector3d(1.0, 2.0, M_PI / 2), 0.0, {segment(0.5, 4.0)});

    REQUIRE(track.duration() == Approx(4.0));
    const auto mid = track.at(2.0);
    REQUIRE(mid.pose.x() == Approx(1.0).margin(1e-9));
    REQUIRE(mid.pose.y() == Approx(3.0));
    REQUIRE(mid.pose.z() == Approx(M_PI / 2));
    REQUIRE(mid.velocity.x() == Approx(0.0).margin(1e-9));
    REQUIRE(mid.velocity.y() == Approx(0.5));
}

TEST_CASE("The target waits at its start before moving and stays still after the end", "[WalkTrackBenchmark][Track]") {
    const Track track(Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, {segment(1.0, 2.0)});

    REQUIRE(track.at(-1.0).pose.x() == Approx(0.0));
    const auto after = track.at(10.0);
    REQUIRE(after.pose.x() == Approx(2.0));
    REQUIRE(after.velocity.norm() == Approx(0.0));
}

TEST_CASE("An arc of curvature 1/r turns the direction by its length over r", "[WalkTrackBenchmark][Track]") {
    // Half a circle of radius 2 from the origin heading +y, turning left: ends 4 m to the left, heading -y
    const double r = 2.0;
    const Track track(Eigen::Vector3d(0.0, 0.0, M_PI / 2), 0.0, {segment(0.5, M_PI * r / 0.5, 0.0, 1.0 / r)});

    const auto end = track.at(track.duration() - 1e-9);
    REQUIRE(end.pose.x() == Approx(-2.0 * r).margin(1e-6));
    REQUIRE(end.pose.y() == Approx(0.0).margin(1e-6));
    REQUIRE(std::remainder(end.direction - 3.0 * M_PI / 2, 2.0 * M_PI) == Approx(0.0).margin(1e-6));
    // Every point is on the circle
    for (double t = 0.0; t < track.duration(); t += 0.5) {
        const auto p = track.at(t).pose;
        REQUIRE((Eigen::Vector2d(p.x(), p.y()) - Eigen::Vector2d(-r, 0.0)).norm() == Approx(r));
    }
}

TEST_CASE("A deceleration stops the target after v²/2a and it stays put", "[WalkTrackBenchmark][Track]") {
    const Track track(Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, {segment(1.2, 6.0, -0.3)});

    // Stops at t = 4 s after 2.4 m
    REQUIRE(track.at(4.0).pose.x() == Approx(2.4));
    REQUIRE(track.at(5.0).pose.x() == Approx(2.4));
    REQUIRE(track.at(5.0).velocity.norm() == Approx(0.0));
    REQUIRE(track.at(2.0).velocity.x() == Approx(0.6));
}

TEST_CASE("A turn at a segment's start doubles back", "[WalkTrackBenchmark][Track]") {
    const Track track(Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, {segment(0.5, 4.0), segment(0.5, 4.0, 0.0, 0.0, M_PI)});

    REQUIRE(track.at(4.0).pose.x() == Approx(2.0));
    REQUIRE(track.at(6.0).pose.x() == Approx(1.0));
    REQUIRE(track.at(6.0).velocity.x() == Approx(-0.5));
    REQUIRE(track.at(8.0).pose.x() == Approx(0.0).margin(1e-9));
}

TEST_CASE("The heading offset turns the asked heading, not the path", "[WalkTrackBenchmark][Track]") {
    const Track track(Eigen::Vector3d(0.0, 0.0, 0.0), M_PI / 2, {segment(0.4, 5.0)});

    const auto p = track.at(2.5);
    REQUIRE(p.pose.x() == Approx(1.0));
    REQUIRE(p.pose.y() == Approx(0.0).margin(1e-9));
    REQUIRE(p.pose.z() == Approx(M_PI / 2));
    REQUIRE(p.direction == Approx(0.0));
}

TEST_CASE("A speed scale covers the same ground faster", "[WalkTrackBenchmark][Track]") {
    const std::vector<Segment> segments = {segment(1.2, 6.0, -0.3), segment(0.5, 3.0, 0.0, 0.8, 0.5)};
    const Track slow(Eigen::Vector3d(0.5, -1.0, 0.3), 0.0, segments);
    const Track fast(Eigen::Vector3d(0.5, -1.0, 0.3), 0.0, segments, 1.5);

    REQUIRE(fast.duration() == Approx(slow.duration() / 1.5));
    for (double f = 0.0; f <= 1.0; f += 0.125) {
        const auto a = slow.at(f * slow.duration());
        const auto b = fast.at(f * fast.duration());
        REQUIRE((a.pose - b.pose).norm() == Approx(0.0).margin(1e-9));
        REQUIRE((b.velocity - 1.5 * a.velocity).norm() == Approx(0.0).margin(1e-9));
    }
}

TEST_CASE("Invalid tracks are refused", "[WalkTrackBenchmark][Track]") {
    REQUIRE_THROWS_AS(Track(Eigen::Vector3d::Zero(), 0.0, {segment(0.5, 1.0)}, 0.0), std::invalid_argument);
    REQUIRE_THROWS_AS(Track(Eigen::Vector3d::Zero(), 0.0, {segment(-0.5, 1.0)}), std::invalid_argument);
    REQUIRE_THROWS_AS(Track(Eigen::Vector3d::Zero(), 0.0, {segment(0.5, -1.0)}), std::invalid_argument);
}
