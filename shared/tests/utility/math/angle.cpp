#include "utility/math/angle.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace utility::math::test {
    using Catch::Matchers::WithinAbs;
    using utility::math::angle::angle_between;

    SCENARIO("Find the angle between two vectors", "[utility][math][angle]") {
        const auto& pi = std::numbers::pi;

        // Define a precision to compare floating point values
        const double precision = 1e-6;

        GIVEN("The x-axis and y-axis") {
            Eigen::Vector3d u{1.0, 0.0, 0.0};
            Eigen::Vector3d v{0.0, 1.0, 0.0};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u, v);

                THEN("The angle should be approximately pi/2") {
                    CHECK_THAT(angle, WithinAbs(pi / 2.0, precision));
                }
            }
        }

        GIVEN("The y-axis and z-axis") {
            Eigen::Vector3d u{0.0, 1.0, 0.0};
            Eigen::Vector3d v{0.0, 0.0, 1.0};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u, v);

                THEN("The angle should be approximately pi/2") {
                    CHECK_THAT(angle, WithinAbs(pi / 2.0, precision));
                }
            }
        }

        GIVEN("Two parallel vectors") {
            Eigen::Vector3d u{1.0, 2.0, 3.0};
            Eigen::Vector3d v{2.0, 4.0, 6.0};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u.normalized(), v.normalized());

                THEN("The angle should be approximately 0") {
                    CHECK_THAT(angle, WithinAbs(0.0, precision));
                }
            }
        }

        GIVEN("Two anti-parallel vectors") {
            Eigen::Vector3d u{1.0, 1.0, 1.0};
            Eigen::Vector3d v{-1.0, -1.0, -1.0};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u.normalized(), v.normalized());

                THEN("The angle should be approximately pi") {
                    CHECK_THAT(angle, WithinAbs(pi, precision));
                }
            }
        }

        GIVEN("Two arbitrary vectors") {
            Eigen::Vector2d u{3.0, 4.0};
            Eigen::Vector2d v{-4.0, 3.0};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u.normalized(), v.normalized());

                THEN("The angle should be approximately pi/2") {
                    CHECK_THAT(angle, WithinAbs(pi / 2.0, precision));
                }
            }
        }

        GIVEN("Two random 2d vectors") {
            // Seed the random number generator
            auto seed = GENERATE(take(10, random(uint64_t(0), std::numeric_limits<uint64_t>::max())));
            // Setup a random number generator
            std::mt19937 gen{seed};
            // Draw samples with uniform probability
            std::uniform_real_distribution<> dist_2pi(0.0, 2.0 * pi);
            std::uniform_real_distribution<> dist_negpi_pi(-pi, pi);

            // Generate a random vector using polar coordinates on the unit circle
            double thetaU = dist_2pi(gen);
            CAPTURE(thetaU);
            Eigen::Vector2d u{std::cos(thetaU), std::sin(thetaU)};

            // Generate a random angle
            double thetaVU = dist_negpi_pi(gen);
            CAPTURE(thetaVU);

            // Construct a second vector using the combined angles to get the other test vector
            Eigen::Vector2d v{std::cos(thetaU + thetaVU), std::sin(thetaU + thetaVU)};

            WHEN("Computing the angle between them") {
                double angle = angle_between(u.normalized(), v.normalized());

                THEN("The angle should be approximately the random angle") {
                    CHECK_THAT(angle, WithinAbs(std::abs(thetaVU), precision));
                }
            }
        }

        GIVEN("Two random 3d vectors") {
            // Seed the random number generator
            auto seed = GENERATE(take(10, random(uint64_t(0), std::numeric_limits<uint64_t>::max())));
            // Setup a random number generator
            std::mt19937 gen{seed};
            // Draw samples with uniform probability
            std::uniform_real_distribution<> dist_2pi(0.0, 2.0 * pi);
            std::uniform_real_distribution<> dist_pi(0.0, pi);
            std::uniform_real_distribution<> dist_negpi_pi(-pi, pi);

            // Generate a random vector using spherical coordinates on the unit sphere
            double phiU = dist_2pi(gen);
            CAPTURE(phiU);
            double thetaU = dist_pi(gen);
            CAPTURE(thetaU);
            Eigen::Vector3d u{
                std::sin(thetaU) * std::cos(phiU),
                std::sin(thetaU) * std::sin(phiU),
                std::cos(thetaU),
            };

            // Generate a random vector to find a random orthogonal axis for rotation using spherical coordinates on the
            // unit sphere
            double phiR = dist_2pi(gen);
            CAPTURE(phiR);
            double thetaR = dist_pi(gen);
            CAPTURE(thetaR);
            Eigen::Vector3d r{
                std::sin(thetaR) * std::cos(phiR),
                std::sin(thetaR) * std::sin(phiR),
                std::cos(thetaR),
            };

            // Generate a random angle for rotation
            double thetaVU = dist_negpi_pi(gen);
            CAPTURE(thetaVU);

            // Construct the rotation matrix
            Eigen::Matrix3d Rvu = Eigen::AngleAxisd(thetaVU, r.cross(u).normalized()).toRotationMatrix();

            // Rotate u by the random rotation matrix to get the other test vector
            Eigen::Vector3d v = Rvu * u;

            WHEN("Computing the angle between them") {
                double angle = angle_between(u.normalized(), v.normalized());

                THEN("The angle should be approximately the random angle of rotation") {
                    CHECK_THAT(angle, WithinAbs(std::abs(thetaVU), precision));
                }
            }
        }
    }

}  // namespace utility::math::test
