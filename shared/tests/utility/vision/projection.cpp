
/*
 * MIT License
 * Modified test to validate OpenCV fisheye projection model
 */

#include "utility/vision/projection.hpp"

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <fmt/format.h>
#include <random>

using Catch::Matchers::WithinAbs;
using utility::vision::project;
using utility::vision::unproject;

using Scalar = double;

struct Lens {
    Eigen::Matrix<Scalar, 4, 1> k;
    Scalar focal_length;
    Eigen::Vector2d centre;
};

template <typename Scalar>
void run_round_trip(const Lens& lens, const Eigen::Matrix<Scalar, 2, 1>& dimensions, const Scalar& margin = 1e-5) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<Scalar> u(Scalar(0), Scalar(1));
    std::uniform_real_distribution<Scalar> v(Scalar(0), Scalar(1));

    for (unsigned i = 0; i < 1000; ++i) {
        Scalar theta = (u(gen) - Scalar(1)) * M_PI_2;
        Scalar phi   = std::acos(Scalar(2) * v(gen) - Scalar(1));

        Eigen::Matrix<Scalar, 3, 1> ray0(std::cos(theta) * std::sin(phi),
                                         std::sin(theta) * std::sin(phi),
                                         std::cos(phi));

        Eigen::Matrix<Scalar, 2, 1> px0  = project(ray0, lens, dimensions);
        Eigen::Matrix<Scalar, 3, 1> ray1 = unproject(px0, lens, dimensions);
        Eigen::Matrix<Scalar, 2, 1> px1  = project(ray1, lens, dimensions);

        INFO(
            fmt::format("ray0: ({:.6f}, {:.6f}, {:.6f})\n"
                        "px0:  ({:.6f}, {:.6f})\n"
                        "ray1: ({:.6f}, {:.6f}, {:.6f})\n"
                        "px1:  ({:.6f}, {:.6f})",
                        ray0.x(),
                        ray0.y(),
                        ray0.z(),
                        px0.x(),
                        px0.y(),
                        ray1.x(),
                        ray1.y(),
                        ray1.z(),
                        px1.x(),
                        px1.y()));

        REQUIRE_THAT(px0.x(), WithinAbs(px1.x(), margin));
        REQUIRE_THAT(px0.y(), WithinAbs(px1.y(), margin));
        REQUIRE_THAT(ray0.x(), WithinAbs(ray1.x(), margin));
        REQUIRE_THAT(ray0.y(), WithinAbs(ray1.y(), margin));
        REQUIRE_THAT(ray0.z(), WithinAbs(ray1.z(), margin));
    }
}

SCENARIO("OpenCV fisheye projection is consistent", "[opencv][fisheye][projection]") {
    WHEN("a 3D ray is projected and unprojected") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);

        Lens lens;
        lens.k << -0.01, 0.001, -0.0001, 0.00001;
        lens.focal_length = 1.0;
        lens.centre       = Eigen::Vector2d(0.0, 0.0);

        THEN("the round-trip error is small") {
            run_round_trip<Scalar>(lens, dimensions);
        }
    }
}
