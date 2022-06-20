#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include <catch2/catch.hpp>
// #define EIGEN_NO_DEBUG // Disable runtime assertions, e.g., bounds checking
#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include <limits>

#include "utility/math/filter/gaussian.hpp"

SCENARIO("logGaussian nominal") {
    GIVEN("Nominal parameters") {
        Eigen::VectorXd x(3);
        x << 1, 2, 3;
        Eigen::VectorXd mu(3);
        mu << 2, 4, 6;
        Eigen::MatrixXd S(3, 3);
        S << 1, 0, 0, 0, 2, 0, 0, 0, 3;

        WHEN("Evaluating l = logGaussian(x,mu,S)") {
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S);
            THEN("l matches expected value") {
                CHECK(l == Approx(-6.04857506884207));
            }
        }

        WHEN("Evaluating l = logGaussian(x,mu,S,g)") {
            Eigen::VectorXd g;
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S, g);

            THEN("l matches expected value") {
                CHECK(l == Approx(-6.04857506884207));
            }

            THEN("g matches expected values") {
                REQUIRE(g.rows() == 3);
                REQUIRE(g.cols() == 1);
                CHECK(g(0) == Approx(1.0));
                CHECK(g(1) == Approx(1.0 / 2.0));
                CHECK(g(2) == Approx(1.0 / 3.0));
            }
        }

        WHEN("Evaluating l = logGaussian(x,mu,S,g,H)") {
            Eigen::VectorXd g;
            Eigen::MatrixXd H;
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S, g, H);

            THEN("l matches expected value") {
                CHECK(l == Approx(-6.04857506884207));
            }

            THEN("g matches expected values") {
                REQUIRE(g.rows() == 3);
                REQUIRE(g.cols() == 1);
                CHECK(g(0) == Approx(1.0));
                CHECK(g(1) == Approx(1.0 / 2.0));
                CHECK(g(2) == Approx(1.0 / 3.0));
            }

            THEN("H matches expected values") {
                REQUIRE(H.rows() == 3);
                REQUIRE(H.cols() == 3);
                CHECK(H(0, 0) == Approx(-1.0));
                CHECK(H(0, 1) == Approx(0.0).margin(1e-10));
                CHECK(H(0, 2) == Approx(0.0).margin(1e-10));
                CHECK(H(1, 0) == Approx(0.0).margin(1e-10));
                CHECK(H(1, 1) == Approx(-1.0 / 4.0));
                CHECK(H(1, 2) == Approx(0.0).margin(1e-10));
                CHECK(H(2, 0) == Approx(0.0).margin(1e-10));
                CHECK(H(2, 1) == Approx(0.0).margin(1e-10));
                CHECK(H(2, 2) == Approx(-1.0 / 9.0));
            }
        }
    }
}

SCENARIO("logGaussian exponential underflow") {
    GIVEN("Parameters that may cause underflow in the exponential") {
        Eigen::VectorXd x(1);
        x << 0;
        Eigen::VectorXd mu(1);
        mu << std::sqrt(350 * std::log(10) / M_PI);  // Approx 16
        Eigen::MatrixXd S(1, 1);
        S << 1.0 / std::sqrt(2 * M_PI);  // Approx 0.4
        REQUIRE(std::exp(-0.5 * S.triangularView<Eigen::Upper>().transpose().solve(x - mu).squaredNorm()) == 0.0);

        WHEN("Evaluating l = logGaussian(x,mu,S)") {
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S);
            THEN("l matches expected value") {
                CHECK(l == Approx(-805.904782547916));
            }
        }
    }
}

SCENARIO("logGaussian determinant underflow") {
    GIVEN("Parameters that may cause determinant underflow") {
        double a           = 1e-4;  // Magnitude of st.dev.
        int n              = 100;   // Dimension
        Eigen::VectorXd x  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd mu = Eigen::VectorXd::Zero(n);
        Eigen::MatrixXd S  = a * Eigen::MatrixXd::Identity(n, n);
        REQUIRE(S.determinant() == 0.0);  // underflow to zero

        WHEN("Evaluating l = logGaussian(x,mu,S)") {
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S);
            THEN("l matches expected value") {
                CHECK(l == Approx(-n * std::log(a) - n / 2.0 * std::log(2 * M_PI)));
            }
        }
    }
}

SCENARIO("logGaussian determinant overflow") {
    GIVEN("Parameters that may cause determinant overflow") {
        double a           = 1e4;  // Magnitude of st.dev.
        int n              = 100;  // Dimension
        Eigen::VectorXd x  = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd mu = Eigen::VectorXd::Zero(n);
        Eigen::MatrixXd S  = a * Eigen::MatrixXd::Identity(n, n);
        REQUIRE(S.determinant() == std::numeric_limits<double>::infinity());  // overflow to infinity

        WHEN("Evaluating l = logGaussian(x,mu,S)") {
            double l = utility::math::filter::gaussian::logGaussian(x, mu, S);
            THEN("l matches expected value") {
                CHECK(l == Approx(-n * std::log(a) - n / 2.0 * std::log(2 * M_PI)));
            }
        }
    }
}
