#define CATCH_CONFIG_MAIN
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <cmath>
#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <random>

using Scalar = double;

namespace utility::vision {

    struct Lens {
        Eigen::Vector4d k;
        Scalar focal_length;
        Eigen::Vector2d centre;
    };

    Eigen::Vector2d project(const Eigen::Vector3d& ray, const Lens& lens) {
        std::vector<cv::Point3d> objectPoints = {cv::Point3d(ray(0), ray(1), ray(2))};
        std::vector<cv::Point2d> imagePoints;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.focal_length,
                     0,
                     lens.centre.x(),
                     0,
                     lens.focal_length,
                     lens.centre.y(),
                     0,
                     0,
                     1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D);

        return Eigen::Vector2d(imagePoints[0].x, imagePoints[0].y);
    }

    Eigen::Vector3d unproject(const Eigen::Vector2d& px, const Lens& lens) {
        std::vector<cv::Point2d> distortedPoints = {cv::Point2d(px(0), px(1))};
        std::vector<cv::Point2d> undistorted;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.focal_length,
                     0,
                     lens.centre.x(),
                     0,
                     lens.focal_length,
                     lens.centre.y(),
                     0,
                     0,
                     1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::undistortPoints(distortedPoints, undistorted, K, D);

        Eigen::Vector3d ray(undistorted[0].x, undistorted[0].y, 1.0);
        return ray.normalized();
    }
}  // namespace utility::vision

using utility::vision::Lens;
using utility::vision::project;
using utility::vision::unproject;

template <typename Scalar>
void run_round_trip(const Lens& lens, const Scalar& margin = 1e-5) {
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
        Eigen::Matrix<Scalar, 2, 1> px0  = project(ray0, lens);
        Eigen::Matrix<Scalar, 3, 1> ray1 = unproject(px0, lens);
        Eigen::Matrix<Scalar, 2, 1> px1  = project(ray1, lens);
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
        REQUIRE(std::abs(ray0.normalized().dot(ray1.normalized())) == Approx(1.0).margin(margin));
    }
}

SCENARIO("OpenCV fisheye projection is consistent", "[opencv][fisheye][projection]") {
    WHEN("a 3D ray is projected and unprojected") {
        Lens lens;
        lens.k << -0.04247437, -0.0076668, 0.00308963, -0.00099647;
        lens.focal_length = 0.3446512345140941 * 1280;  // pixels
        lens.centre =
            Eigen::Vector2d(1280 * 0.5 + 0.005418370784106563 * 1280, 1024 * 0.5 - 0.08661520006894258 * 1280);
        THEN("the round-trip error is small") {
            run_round_trip<Scalar>(lens);
        }
    }
}
