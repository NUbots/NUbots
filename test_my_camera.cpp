#define CATCH_CONFIG_MAIN
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <cmath>
#include <fmt/format.h>
#include <random>

using Scalar = double;

// Inline all the projection functions to avoid dependencies
namespace utility::vision {
    template <typename T, typename Lens>
    inline T distort(const T& theta, const Lens& lens) {
        const auto& k  = lens.k;
        const T theta2 = theta * theta;
        const T theta4 = theta2 * theta2;
        const T theta6 = theta4 * theta2;
        const T theta8 = theta4 * theta4;
        return theta * (T(1) + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
    }

    template <typename T, typename Lens>
    T undistort(const T& theta_d, const Lens& lens) {
        T theta = theta_d;
        for (int i = 0; i < 5; ++i) {
            T theta2 = theta * theta;
            T theta4 = theta2 * theta2;
            T theta6 = theta4 * theta2;
            T theta8 = theta4 * theta4;
            T f = theta * (T(1) + lens.k[0] * theta2 + lens.k[1] * theta4 + lens.k[2] * theta6 + lens.k[3] * theta8)
                  - theta_d;
            T df  = (T(1) + 3 * lens.k[0] * theta2 + 5 * lens.k[1] * theta4 + 7 * lens.k[2] * theta6
                    + 9 * lens.k[3] * theta8);
            theta = theta - f / df;
        }
        return theta;
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time, typename Lens>
    Eigen::Matrix<T, 2, 1> project(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const Lens& lens,
        const Eigen::Matrix<T, 2, 1>& dimensions) {
        const T theta     = std::acos(ray.x());
        const T theta_d   = distort(theta, lens);
        const T sin_theta = std::sqrt(1 - ray.x() * ray.x());
        Eigen::Matrix<T, 2, 1> screen =
            ray.x() >= T(1) ? Eigen::Matrix<T, 2, 1>::Zero()
                            : Eigen::Matrix<T, 2, 1>(theta_d * ray.y() / sin_theta, theta_d * ray.z() / sin_theta);
        return (dimensions * T(0.5)) - screen - lens.centre.template cast<T>();
    }

    template <typename T, typename Lens>
    Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& px,
                                     const Lens& lens,
                                     const Eigen::Matrix<T, 2, 1>& dimensions) {
        Eigen::Matrix<T, 2, 1> screen = (dimensions * T(0.5)) - px - lens.centre.template cast<T>();
        const T r_d                   = screen.norm();
        if (r_d == T(0)) {
            return Eigen::Matrix<T, 3, 1>::UnitX();
        }
        const T theta     = undistort(r_d, lens);
        const T sin_theta = std::sin(theta);
        return Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * screen.x() / r_d, sin_theta * screen.y() / r_d);
    }
}  // namespace utility::vision

using utility::vision::project;
using utility::vision::unproject;

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
            fmt::format("ray0: ({:.6f}, {:.6f}, {:.6f})\npx0: ({:.6f}, {:.6f})\nray1: ({:.6f}, {:.6f}, {:.6f})\npx1: "
                        "({:.6f}, {:.6f})",
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
        REQUIRE(Approx(px1.x()).margin(margin) == px0.x());
        REQUIRE(Approx(px1.y()).margin(margin) == px0.y());
        REQUIRE(Approx(ray1.x()).margin(margin) == ray0.x());
        REQUIRE(Approx(ray1.y()).margin(margin) == ray0.y());
        REQUIRE(Approx(ray1.z()).margin(margin) == ray0.z());
    }
}

SCENARIO("OpenCV fisheye projection is consistent", "[opencv][fisheye][projection]") {
    WHEN("a 3D ray is projected and unprojected") {
        const Eigen::Matrix<Scalar, 2, 1> dimensions(1920, 1200);
        Lens lens;
        lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;
        lens.focal_length = 0.3446512345140941;
        lens.centre       = Eigen::Vector2d(0.005418370784106563, -0.08661520006894258);
        THEN("the round-trip error is small") {
            run_round_trip<Scalar>(lens, dimensions);
        }
    }
}
