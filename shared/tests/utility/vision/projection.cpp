/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
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

#include "message/input/Lens.hpp"

using message::input::Lens;
using utility::vision::project;
using utility::vision::unproject;

using Catch::Matchers::WithinAbs;

using Scalar = double;

Lens create_lens(const std::string& projection,
                 const float& focal_length,
                 const float& fov,
                 const Eigen::Vector2f& centre,
                 const Eigen::Vector2f& k,
                 const Eigen::Vector2f& dimensions,
                 const Eigen::Vector2f& full_dimensions) {

    // Set the lens parameters from configuration
    return Lens{projection,
                // Un-normalise focal length
                focal_length * full_dimensions.x(),
                fov,
                // Recentre the centre
                (centre - (full_dimensions - dimensions - centre)) * 0.5f,
                // Un-normalise the distortion parameters
                k.cwiseQuotient(Eigen::Vector2f(std::pow(full_dimensions.x(), 2), std::pow(full_dimensions.x(), 4)))};
}

Lens create_normalised_lens(const std::string& projection,
                            const float& focal_length,
                            const float& fov,
                            const Eigen::Vector2f& centre,
                            const Eigen::Vector2f& k,
                            const Eigen::Vector2f& dimensions,
                            const Eigen::Vector2f& full_dimensions) {
    Lens lens = create_lens(projection, focal_length, fov, centre, k, dimensions, full_dimensions);

    // Set the lens parameters from configuration
    return Lens{lens.projection,
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
void run_round_trip(const Lens& lens,
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
        const Lens lens =
            create_normalised_lens(Lens::Projection("EQUISOLID"),                                  // projection
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
        const Lens lens =
            create_normalised_lens(Lens::Projection("EQUIDISTANT"),                                // projection
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
        const Lens lens = create_normalised_lens(Lens::Projection("RECTILINEAR"),  // projection
                                                 1.362315898710812,                // focal length
                                                 41.0f * M_PI / 180.0f,            // field of view
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
        const Lens lens = create_lens(Lens::Projection("EQUISOLID"),                                  // projection
                                      0.20980090703929113f,                                           // focal length
                                      183.0f * M_PI / 180.0f,                                         // field of view
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
        const Lens lens = create_lens(Lens::Projection("EQUIDISTANT"),                                // projection
                                      0.20980090703929113f,                                           // focal length
                                      183.0f * M_PI / 180.0f,                                         // field of view
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
        const Lens lens = create_lens(Lens::Projection("RECTILINEAR"),                              // projection
                                      1.362315898710812f,                                           // focal length
                                      41.0f * M_PI / 180.0f,                                        // field of view
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
