#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include <catch.hpp>
// #define EIGEN_NO_DEBUG // Disable runtime assertions, e.g., bounds checking
#include <Eigen/Core>
#include <cmath>

#include "utility/math/filter/fmin.hpp"

TEST_CASE("TRS: Return Newton step when it is inside trust region") {
    Eigen::VectorXd g(4);
    g << 1, 1, 1, 1;

    Eigen::MatrixXd H(4, 4);
    H << 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4;

    double Delta = 2;

    Eigen::VectorXd p(4);
    int retval = trs(H, g, Delta, p);

    CHECK(retval == 0);

    Eigen::VectorXd pNewton = -H.llt().solve(g);
    CHECK(p(0) == Approx(pNewton(0)));
    CHECK(p(1) == Approx(pNewton(1)));
    CHECK(p(2) == Approx(pNewton(2)));
    CHECK(p(3) == Approx(pNewton(3)));
}

TEST_CASE("TRS: Step length equals trust region radius when Newton step outside trust region") {
    Eigen::VectorXd g(4);
    g << 1, 1, 1, 1;

    Eigen::MatrixXd H(4, 4);
    H << 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4;

    Eigen::VectorXd pNewton = -H.llt().solve(g);
    double Delta            = 0.9 * pNewton.norm();

    Eigen::VectorXd p(4);
    int retval = trs(H, g, Delta, p);

    CHECK(retval == 0);
    CHECK(p.norm() == Approx(Delta));
}

TEST_CASE("TRS: Step length equals trust region radius when nonconvex") {
    Eigen::VectorXd g(4);
    g << 1, 1, 1, 1;

    Eigen::MatrixXd H(4, 4);
    H << -2, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    double Delta = 2;

    Eigen::VectorXd p(4);
    int retval = trs(H, g, Delta, p);

    CHECK(retval == 0);
    CHECK(p.norm() == Approx(Delta));
}

TEST_CASE("TRS: Step length equals trust region radius when nonconvex (hard case)") {
    Eigen::VectorXd g(4);
    g << 0, 1, 1, 1;

    Eigen::MatrixXd H(4, 4);
    H << -2, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    double Delta = 2;

    Eigen::VectorXd p(4);
    int retval = trs(H, g, Delta, p);

    CHECK(retval == 0);
    CHECK(p.norm() == Approx(Delta));
}
