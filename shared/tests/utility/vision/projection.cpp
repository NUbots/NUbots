/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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

#include "utility/vision/projection.hpp"

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fmt/format.h>
#include <random>
#include <string>

#include "message/input/Image.hpp"

using message::input::Image;
using utility::vision::project;
using utility::vision::project_pixel;
using utility::vision::unproject;
using utility::vision::unproject_pixel;

using Catch::Matchers::WithinAbs;

using Scalar = double;

Image::Lens create_lens(const std::string& projection,
                        const float& focal_length,
                        const float& fov,
                        const Eigen::Vector2f& centre,
                        const Eigen::Vector2f& k,
                        const Eigen::Vector2f& dimensions,
                        const Eigen::Vector2f& full_dimensions) {

    // Set the lens parameters from configuration
    return Image::Lens{
        projection,
        // Un-normalise focal length
        focal_length * full_dimensions.x(),
        fov,
        // Recentre the centre
        (centre - (full_dimensions - dimensions - centre)) * 0.5f,
        // Un-normalise the distortion parameters
        k.cwiseQuotient(Eigen::Vector2f(std::pow(full_dimensions.x(), 2), std::pow(full_dimensions.x(), 4)))};
}

Image::Lens create_normalised_lens(const std::string& projection,
                                   const float& focal_length,
                                   const float& fov,
                                   const Eigen::Vector2f& centre,
                                   const Eigen::Vector2f& k,
                                   const Eigen::Vector2f& dimensions,
                                   const Eigen::Vector2f& full_dimensions) {
    Image::Lens lens = create_lens(projection, focal_length, fov, centre, k, dimensions, full_dimensions);

    // Set the lens parameters from configuration
    return Image::Lens{lens.projection,
                       // Normalise focal length
                       lens.focal_length / dimensions.x(),
                       fov,
                       // Normalise the centre
                       lens.centre / dimensions.x(),
                       // Adjust the distortion parameters for the new width units
                       k.cwiseProduct(Eigen::Vector2f(std::pow(dimensions.x() / full_dimensions.x(), 2),
                                                      std::pow(dimensions.x() / full_dimensions.x(), 4)))};
}

template <typename Scalar>
void run_round_trip(const Image::Lens& lens,
                    const Eigen::Matrix<Scalar, 2, 1>& dimensions,
                    const bool& normalised,
                    const Scalar& margin = 1e-5) {
    std::random_device rd;   // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());  // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<Scalar> u(Scalar(0), Scalar(1));
    std::uniform_real_distribution<Scalar> v(Scalar(0), Scalar(1));

    const Eigen::Matrix<Scalar, 2, 1> dims = normalised ? dimensions / dimensions.x() : dimensions;

    for (unsigned i = 0; i < 1000; ++i) {
        // Pick a random point on the unit half-sphere
        // x will always be one (radial distance is always 1 in the unit sphere)
        // θ and φ are picked using the scheme described here
        //     https://mathworld.wolfram.com/SpherePointPicking.html
        // Since we are interested in the unit half-sphere centered on the positive x-axis we need
        // to restrict θ to be in the range [-π/2, π/2]
        // -----------------------------------------------------------------------------------------
        // The above is true for a 180° FOV
        // For different sized FOVs we need to restrict both θ and φ so that we get the
        // appropriate snapshot of the unit sphere
        // If FOV < π
        //  -FOV/2 <= θ <= FOV/2
        //  π/2 - FOV/2 <= φ <= π/2 + FOV/2
        // If FOV = π
        //  -π/2 <= θ <= π/2
        //  0 <= φ <= π
        // If FOV > π
        //  -FOV/2 <= θ <= FOV/2
        //  0 <= φ <= π

        Scalar theta = (u(gen) - Scalar(1)) * lens.fov * 0.5;
        Scalar phi   = std::acos(Scalar(2) * v(gen) - Scalar(1));

        // Map φ from [0, π] → [π/2 - FOV/2, π/2 + FOV/2]
        if (lens.fov < M_PI) {
            const Scalar range_start = (M_PI_2 - lens.fov * 0.5);
            const Scalar range_end   = (M_PI_2 + lens.fov * 0.5);
            phi                      = ((range_end - range_start) / M_PI) * phi + range_start;
        }

        Eigen::Matrix<Scalar, 3, 1> ray0(std::cos(theta) * std::sin(phi),
                                         std::sin(theta) * std::sin(phi),
                                         std::cos(phi));

        Eigen::Matrix<Scalar, 2, 1> px0  = project(ray0, lens, dims);
        Eigen::Matrix<Scalar, 3, 1> ray1 = unproject(px0, lens, dims);
        Eigen::Matrix<Scalar, 2, 1> px1  = project(ray1, lens, dims);

        INFO(
            fmt::format("Testing:"
                        "\n\tθ: {:.2f}"
                        "\n\tφ: {:.2f}"
                        "\n\tray0: ({:.2f}, {:.2f}, {:.2f}) ({:.2f})"
                        "\n\tpx0: ({:.2f}, {:.2f})"
                        "\n\tray0: ({:.2f}, {:.2f}, {:.2f}) ({:.2f})"
                        "\n\tpx1: ({:.2f}, {:.2f})",
                        theta,
                        phi,
                        ray0.x(),
                        ray0.y(),
                        ray0.z(),
                        ray0.norm(),
                        px0.x(),
                        px0.y(),
                        ray1.x(),
                        ray1.y(),
                        ray1.z(),
                        ray1.norm(),
                        px1.x(),
                        px1.y()));
        REQUIRE_THAT(px0.x(), WithinAbs(px1.x(), margin));
        REQUIRE_THAT(px0.y(), WithinAbs(px1.y(), margin));
        REQUIRE_THAT(ray0.x(), WithinAbs(ray1.x(), margin));
        REQUIRE_THAT(ray0.y(), WithinAbs(ray1.y(), margin));
        REQUIRE_THAT(ray0.z(), WithinAbs(ray1.z(), margin));
    }
}

SCENARIO("pixel and unit vector projections are accurate", "[utility][vision][projection]") {
    WHEN("width-normalised equisolid projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);
        const Image::Lens lens =
            create_normalised_lens(Image::Lens::Projection("EQUISOLID"),                           // projection
                                   0.20980090703929113f,                                           // focal length
                                   183.0f * M_PI / 180.0f,                                         // field of view
                                   Eigen::Vector2f(-0.017560194004901337, -0.015374040186510488),  // centre
                                   Eigen::Vector2f(-0.1118031941066955, -0.003381828624269054),    // k
                                   dimensions.cast<float>(),
                                   dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Normalised equisolid tests");
            run_round_trip<Scalar>(lens, dimensions, true);
        }
    }

    WHEN("width-normalised equidistant projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);
        const Image::Lens lens =
            create_normalised_lens(Image::Lens::Projection("EQUIDISTANT"),                         // projection
                                   0.20980090703929113f,                                           // focal length
                                   183.0f * M_PI / 180.0f,                                         // field of view
                                   Eigen::Vector2f(-0.017560194004901337, -0.015374040186510488),  // centre
                                   Eigen::Vector2f(-0.1118031941066955, -0.003381828624269054),    // k
                                   dimensions.cast<float>(),
                                   dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Normalised equidistant tests");
            run_round_trip<Scalar>(lens, dimensions, true);
        }
    }

    WHEN("width-normalised rectlinear projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(2448, 2048);
        const Eigen::Matrix<Scalar, 2, 1> full_dimensions(2448, 2048);
        const Image::Lens lens =
            create_normalised_lens(Image::Lens::Projection("RECTILINEAR"),                       // projection
                                   1.362315898710812,                                            // focal length
                                   41.0f * M_PI / 180.0f,                                        // field of view
                                   Eigen::Vector2f(-0.0332563868776798, -0.035537119404220684),  // centre
                                   Eigen::Vector2f(0.08337106835599951, 0.008852751521405857),   // k
                                   dimensions.cast<float>(),
                                   full_dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Normalised rectilinear tests");
            run_round_trip<Scalar>(lens, dimensions, true);
        }
    }

    WHEN("equisolid projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);
        const Image::Lens lens = create_lens(Image::Lens::Projection("EQUISOLID"),  // projection
                                             0.20980090703929113f,                  // focal length
                                             183.0f * M_PI / 180.0f,                // field of view
                                             Eigen::Vector2f(-0.017560194004901337, -0.015374040186510488),  // centre
                                             Eigen::Vector2f(-0.1118031941066955, -0.003381828624269054),    // k
                                             dimensions.cast<float>(),
                                             dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Un-normalised equisolid tests");
            run_round_trip<Scalar>(lens, dimensions, false, 1e-4);
        }
    }

    WHEN("equidistant projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);
        const Image::Lens lens = create_lens(Image::Lens::Projection("EQUIDISTANT"),  // projection
                                             0.20980090703929113f,                    // focal length
                                             183.0f * M_PI / 180.0f,                  // field of view
                                             Eigen::Vector2f(-0.017560194004901337, -0.015374040186510488),  // centre
                                             Eigen::Vector2f(-0.1118031941066955, -0.003381828624269054),    // k
                                             dimensions.cast<float>(),
                                             dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Un-normalised equidistant tests");
            run_round_trip<Scalar>(lens, dimensions, false, 1e-4);
        }
    }

    WHEN("rectlinear projections are roundtripped") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(2448, 2048);
        const Eigen::Matrix<Scalar, 2, 1> full_dimensions(2448, 2048);
        const Image::Lens lens = create_lens(Image::Lens::Projection("RECTILINEAR"),  // projection
                                             1.362315898710812f,                      // focal length
                                             41.0f * M_PI / 180.0f,                   // field of view
                                             Eigen::Vector2f(-0.0332563868776798, -0.035537119404220684),  // centre
                                             Eigen::Vector2f(0.08337106835599951, 0.008852751521405857),   // k
                                             dimensions.cast<float>(),
                                             full_dimensions.cast<float>());

        THEN("the error is small") {
            INFO("Un-normalised rectilinear tests");
            run_round_trip<Scalar>(lens, dimensions, false, 2e-2);
        }
    }
}

namespace {

    /// @brief A ray at incidence theta, azimuth phi, in the camera frame (x optical, y left, z up).
    Eigen::Vector3d ray_at(const double& theta_deg, const double& phi_deg) {
        const double theta = theta_deg * M_PI / 180.0;
        const double phi   = phi_deg * M_PI / 180.0;
        return Eigen::Vector3d(std::cos(theta), std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi))
            .normalized();
    }

    /// @brief The simulated camera: an undistorted 90 deg rectilinear pinhole at 640x480.
    ///
    /// NUWebots protos/robot/nugus/nugus.proto sets the Camera node's `spherical FALSE` and
    /// `fieldOfView 1.5707`, whence focal_length = (640/2)/tan(1.5707/2)/640 = 0.5.
    Image::Lens webots_lens() {
        return Image::Lens{Image::Lens::Projection("RECTILINEAR"),
                           0.5f,
                           1.5707f,
                           Eigen::Vector2f::Zero(),
                           Eigen::Vector2f::Zero()};
    }

    /// @brief A real robot's fisheye: a Lensagon BF10M19828S118C calibration at 1280x1024.
    ///
    /// The distortion coefficients are optional so that a caller wanting an exact project/unproject
    /// inverse (rather than the ~0.2 px the distortion polynomial pair is accurate to) can drop them.
    Image::Lens fisheye_lens(const bool& distorted = true) {
        return Image::Lens{
            Image::Lens::Projection("EQUIDISTANT"),
            0.34690945742400775f,
            183.0f * float(M_PI) / 180.0f,
            Eigen::Vector2f(0.02072339174622414f, -0.0011612242293956145f),
            distorted ? Eigen::Vector2f(0.38553542593448015f, 0.1498415334589703f) : Eigen::Vector2f::Zero()};
    }

}  // namespace

SCENARIO("pixel-space projection matches the width-normalised projection", "[utility][vision][projection]") {
    GIVEN("A lens calibration and the image size in pixels") {
        const Image::Lens lens = fisheye_lens();
        const Eigen::Matrix<Scalar, 2, 1> pixels(1280, 1024);
        const Eigen::Matrix<Scalar, 2, 1> normalised = pixels / pixels.x();

        THEN("project_pixel is project scaled up by the image width") {
            const Eigen::Vector3d ray                  = ray_at(35.0, 20.0);
            const Eigen::Matrix<Scalar, 2, 1> px       = project_pixel(ray, lens, pixels);
            const Eigen::Matrix<Scalar, 2, 1> expected = project(ray, lens, normalised) * pixels.x();
            REQUIRE_THAT(px.x(), WithinAbs(expected.x(), 1e-9));
            REQUIRE_THAT(px.y(), WithinAbs(expected.y(), 1e-9));
        }

        THEN("unproject_pixel is unproject scaled down by the image width") {
            const Eigen::Matrix<Scalar, 2, 1> px(910.0, 300.0);
            const Eigen::Vector3d ray              = unproject_pixel(px, lens, pixels);
            const Eigen::Matrix<Scalar, 2, 1> px_n = px / pixels.x();
            const Eigen::Vector3d expected         = unproject(px_n, lens, normalised);
            REQUIRE_THAT((ray - expected).norm(), WithinAbs(0.0, 1e-9));
        }
    }
}

SCENARIO("pixel projections round-trip", "[utility][vision][projection]") {
    GIVEN("A distortion-free equidistant fisheye") {
        const Image::Lens lens = fisheye_lens(false);
        const Eigen::Matrix<Scalar, 2, 1> pixels(1280, 1024);

        THEN("project_pixel then unproject_pixel recovers each ray") {
            for (double theta = 0.0; theta <= 70.0; theta += 10.0) {
                for (double phi = 0.0; phi < 360.0; phi += 45.0) {
                    const Eigen::Vector3d ray  = ray_at(theta, phi);
                    const Eigen::Vector3d back = unproject_pixel(project_pixel(ray, lens, pixels), lens, pixels);
                    REQUIRE_THAT((back - ray).norm(), WithinAbs(0.0, 1e-9));
                }
            }
        }
    }

    GIVEN("The webots pinhole") {
        const Image::Lens lens = webots_lens();
        const Eigen::Matrix<Scalar, 2, 1> pixels(640, 480);

        THEN("project_pixel and unproject_pixel are exact inverses (no distortion to approximate)") {
            for (double theta = 0.0; theta <= 55.0; theta += 5.0) {
                for (double phi = 0.0; phi < 360.0; phi += 45.0) {
                    const Eigen::Vector3d ray  = ray_at(theta, phi);
                    const Eigen::Vector3d back = unproject_pixel(project_pixel(ray, lens, pixels), lens, pixels);
                    REQUIRE_THAT((back - ray).norm(), WithinAbs(0.0, 1e-9));
                }
            }
        }
    }
}

SCENARIO("the projection model is a property of the camera, not a constant", "[utility][vision][projection]") {
    GIVEN("The webots pinhole") {
        const Image::Lens lens = webots_lens();
        const Eigen::Matrix<Scalar, 2, 1> pixels(640, 480);

        THEN("the horizontal edges of the image sit at +/- 45 deg") {
            const Eigen::Vector3d left  = unproject_pixel(Eigen::Vector2d(0.0, 240.0), lens, pixels);
            const Eigen::Vector3d right = unproject_pixel(Eigen::Vector2d(640.0, 240.0), lens, pixels);
            REQUIRE_THAT(std::atan2(left.y(), left.x()), WithinAbs(M_PI / 4.0, 1e-9));
            REQUIRE_THAT(std::atan2(right.y(), right.x()), WithinAbs(-M_PI / 4.0, 1e-9));
        }

        THEN("a ray at the horizon stays finite and lands outside the image") {
            // tan(90 deg) is infinite; a bounds check can only reject the pixel if the projection
            // stayed a number.
            const Eigen::Vector2d px = project_pixel(Eigen::Vector3d(1e-4, 1.0, 0.0).normalized(), lens, pixels);
            REQUIRE(std::isfinite(px.x()));
            REQUIRE(std::isfinite(px.y()));
            REQUIRE_FALSE((px.x() >= 0.0 && px.x() < pixels.x() && px.y() >= 0.0 && px.y() < pixels.y()));
        }
    }

    GIVEN("The same ray through the fisheye and the pinhole") {
        THEN("they disagree by degrees, so the calibration cannot be shared") {
            // Replaying a webots frame through a fisheye calibration does not merely shift the
            // re-projection, it bends it -- which is why the projection travels with the lens.
            const Eigen::Matrix<Scalar, 2, 1> fisheye_px(1280, 1024);
            const Eigen::Matrix<Scalar, 2, 1> pinhole_px(640, 480);
            const Eigen::Vector3d ray = ray_at(40.0, 0.0);
            const double r_fisheye    = project_pixel(ray, fisheye_lens(), fisheye_px).x() / fisheye_px.x() - 0.5;
            const double r_pinhole    = project_pixel(ray, webots_lens(), pinhole_px).x() / pinhole_px.x() - 0.5;
            REQUIRE(std::abs(r_fisheye - r_pinhole) > 0.05);
        }
    }

    GIVEN("An equisolid lens") {
        THEN("it follows r = 2 f sin(theta/2) rather than the equidistant law") {
            // The Image message carries three projections, and a round-trip test cannot tell them
            // apart: project and unproject would agree with each other on the wrong model. Pin the
            // law itself so treating EQUISOLID as equidistant is a failure rather than a silence.
            const double f     = 0.34690945742400775;
            const double theta = 50.0 * M_PI / 180.0;
            const double r     = utility::vision::equisolid::r(theta, f);
            REQUIRE_THAT(r, WithinAbs(2.0 * f * std::sin(0.5 * theta), 1e-12));
            REQUIRE_THAT(utility::vision::equisolid::theta(r, f), WithinAbs(theta, 1e-12));
            REQUIRE(std::abs(r - utility::vision::equidistant::r(theta, f)) > 1e-3);
        }

        THEN("a radius past its horizon saturates instead of going nan") {
            // asin is undefined past r = 2f: a pixel outside the lens circle must still unproject to
            // something a bounds check can reject.
            const double f = 0.34690945742400775;
            REQUIRE_THAT(utility::vision::equisolid::theta(4.0 * f, f), WithinAbs(M_PI, 1e-12));
        }
    }
}
