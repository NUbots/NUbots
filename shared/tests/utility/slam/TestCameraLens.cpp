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

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

#include "utility/slam/camera/CameraLens.hpp"

using Catch::Matchers::WithinAbs;

using utility::slam::camera::CameraLens;

namespace {

    /// @brief The simulated camera: an undistorted 90 deg pinhole at 640x480.
    ///
    /// NUWebots protos/robot/nugus/nugus.proto sets the Camera node's `spherical FALSE` and
    /// `fieldOfView 1.5707`, whence focal_length = (640/2)/tan(1.5707/2)/640 = 0.5. This is what a
    /// webots Image message delivers through CameraLens::fromLens.
    CameraLens webots_lens() {
        CameraLens l;
        l.width       = 640.0;
        l.height      = 480.0;
        l.projection  = CameraLens::Projection::RECTILINEAR;
        l.focalLength = 0.5;
        l.centre      = Eigen::Vector2d::Zero();
        l.k           = Eigen::Vector2d::Zero();
        return l;
    }

    /// @brief A ray at incidence theta, azimuth phi, in the camera frame (x optical, y left, z up).
    Eigen::Vector3d ray_at(double theta_deg, double phi_deg) {
        const double theta = theta_deg * M_PI / 180.0;
        const double phi   = phi_deg * M_PI / 180.0;
        return Eigen::Vector3d(std::cos(theta), std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi))
            .normalized();
    }

}  // namespace

SCENARIO("CameraLens project/unproject round-trip", "[slam][camera]") {
    GIVEN("A distortion-free equidistant fisheye") {
        CameraLens lens;
        lens.k = Eigen::Vector2d::Zero();  // Pure equidistant: project/unproject are exact inverses

        THEN("project then unproject recovers each ray") {
            for (double theta = 0.0; theta <= 70.0; theta += 10.0) {
                for (double phi = 0.0; phi < 360.0; phi += 45.0) {
                    const Eigen::Vector3d ray  = ray_at(theta, phi);
                    const Eigen::Vector3d back = lens.unproject(lens.project(ray));
                    REQUIRE_THAT((back - ray).norm(), WithinAbs(0.0, 1e-9));
                }
            }
        }
    }

    GIVEN("The webots pinhole") {
        const CameraLens lens = webots_lens();

        THEN("project and unproject are exact inverses (no distortion to approximate)") {
            for (double theta = 0.0; theta <= 55.0; theta += 5.0) {
                for (double phi = 0.0; phi < 360.0; phi += 45.0) {
                    const Eigen::Vector3d ray  = ray_at(theta, phi);
                    const Eigen::Vector3d back = lens.unproject(lens.project(ray));
                    REQUIRE_THAT((back - ray).norm(), WithinAbs(0.0, 1e-9));
                }
            }
        }
    }
}

SCENARIO("The projection model is a property of the camera, not a constant", "[slam][camera]") {
    GIVEN("The webots pinhole") {
        const CameraLens lens = webots_lens();

        THEN("the horizontal edges of the image sit at +/- 45 deg") {
            const Eigen::Vector3d left  = lens.unproject(Eigen::Vector2d(0.0, lens.height * 0.5));
            const Eigen::Vector3d right = lens.unproject(Eigen::Vector2d(lens.width, lens.height * 0.5));
            REQUIRE_THAT(std::atan2(left.y(), left.x()), WithinAbs(M_PI / 4.0, 1e-9));
            REQUIRE_THAT(std::atan2(right.y(), right.x()), WithinAbs(-M_PI / 4.0, 1e-9));
        }

        THEN("horizontalHalfFov measures 45 deg rather than assuming a fisheye") {
            REQUIRE_THAT(lens.horizontalHalfFov() * 180.0 / M_PI, WithinAbs(45.0, 1e-6));
        }

        THEN("a ray at the horizon stays finite and lands outside the image") {
            // tan(90 deg) is infinite; the model clamps rather than emitting nan, so inImage() can
            // still reject the pixel.
            const Eigen::Vector2d px = lens.project(Eigen::Vector3d(1e-4, 1.0, 0.0).normalized());
            REQUIRE(std::isfinite(px.x()));
            REQUIRE(std::isfinite(px.y()));
            REQUIRE_FALSE(lens.inImage(px));
        }
    }

    GIVEN("The default fisheye calibration") {
        const CameraLens lens;

        THEN("it is equidistant and much wider than the simulated camera") {
            REQUIRE(lens.projection == CameraLens::Projection::EQUIDISTANT);
            REQUIRE(lens.horizontalHalfFov() > 80.0 * M_PI / 180.0);
        }
    }

    GIVEN("The same ray through the fisheye and the pinhole") {
        THEN("they disagree by degrees, so the calibration cannot be shared") {
            // Replaying a webots frame through a fisheye calibration does not merely shift the
            // re-projection, it bends it -- which is why the projection travels with the lens.
            const CameraLens fisheye;
            const CameraLens pinhole  = webots_lens();
            const Eigen::Vector3d ray = ray_at(40.0, 0.0);
            const double r_fisheye    = (fisheye.project(ray).x() - fisheye.width * 0.5) / fisheye.width;
            const double r_pinhole    = (pinhole.project(ray).x() - pinhole.width * 0.5) / pinhole.width;
            REQUIRE(std::abs(r_fisheye - r_pinhole) > 0.05);
        }
    }

    GIVEN("An equisolid lens") {
        CameraLens lens;
        lens.projection = CameraLens::Projection::EQUISOLID;
        lens.k          = Eigen::Vector2d::Zero();

        THEN("it follows r = 2 f sin(theta/2) rather than the equidistant law") {
            // The Image message carries three projections; treating EQUISOLID as equidistant would
            // be silently wrong rather than unsupported.
            const double theta = 50.0 * M_PI / 180.0;
            REQUIRE_THAT(lens.radiusForAngle(theta), WithinAbs(2.0 * lens.focalLength * std::sin(0.5 * theta), 1e-12));
            REQUIRE_THAT(lens.angleForRadius(lens.radiusForAngle(theta)), WithinAbs(theta, 1e-12));
        }
    }
}
