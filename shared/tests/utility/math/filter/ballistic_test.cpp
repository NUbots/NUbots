#include "ballistic.h"

#include <Eigen/Core>
#include <catch.hpp>

TEST_CASE("Ballistic: Calling process model") {

    int nx = 3;
    Eigen::VectorXd x(nx), f;
    Eigen::VectorXd u;
    // x - [3 x 1]:
    x << 14000, -450, 0.0005;

    BallisticProcessModel pm;
    BallisticParameters param;

    WHEN("Four arguments") {
        pm(x, u, param, f);
    }

    WHEN("More than four arguments") {
        Eigen::MatrixXd SQ;

        WHEN("Five arguments") {
            pm(x, u, param, f, SQ);
        }

        WHEN("Six arguments") {
            Eigen::MatrixXd A;

            pm(x, u, param, f, SQ, A);

            REQUIRE(A.size() > 0);
            REQUIRE(A.rows() == 3);
            REQUIRE(A.cols() == 3);

            // A(:,1)
            CHECK(A(0, 0) == Approx(0));
            CHECK(A(1, 0) == Approx(-0.00172994));
            CHECK(A(2, 0) == Approx(0));

            // A(:,2)
            CHECK(A(0, 1) == Approx(1));
            CHECK(A(1, 1) == Approx(-0.0547733));
            CHECK(A(2, 1) == Approx(0));

            // A(:,3)
            CHECK(A(0, 2) == Approx(0));
            CHECK(A(1, 2) == Approx(24648));
            CHECK(A(2, 2) == Approx(0));
        }

        REQUIRE(SQ.size() > 0);
        REQUIRE(SQ.rows() == 3);
        REQUIRE(SQ.cols() == 3);

        // SQ(:,1)
        CHECK(SQ(0, 0) == Approx(0));
        CHECK(SQ(1, 0) == Approx(0));
        CHECK(SQ(2, 0) == Approx(0));

        // SQ(:,2)
        CHECK(SQ(0, 1) == Approx(0));
        CHECK(SQ(1, 1) == Approx(1e-10));
        CHECK(SQ(2, 1) == Approx(0));

        // SQ(:,3)
        CHECK(SQ(0, 2) == Approx(0));
        CHECK(SQ(1, 2) == Approx(0));
        CHECK(SQ(2, 2) == Approx(5e-06));
    }
    REQUIRE(f.size() > 0);
    REQUIRE(f.rows() == 3);
    REQUIRE(f.cols() == 1);

    // f(:,1)
    CHECK(f(0, 0) == Approx(-450));
    CHECK(f(1, 0) == Approx(2.51399));
    CHECK(f(2, 0) == Approx(0));
}


TEST_CASE("Ballistic: Calling measurement model") {
    int nx = 3;
    // int ny = 1;
    Eigen::VectorXd x(nx);
    Eigen::VectorXd u;
    Eigen::VectorXd h;
    // x - [3 x 1]:
    x << 13955, -449.745, 0.0005;

    BallisticMeasurementModel mm;
    BallisticParameters param;

    WHEN("Four arguments") {
        mm(x, u, param, h);
    }

    WHEN("More than four arguments") {
        Eigen::MatrixXd SR;

        WHEN("Five arguments") {

            mm(x, u, param, h, SR);
        }

        WHEN("Six arguments") {
            Eigen::MatrixXd C;
            mm(x, u, param, h, SR, C);

            REQUIRE(C.size() > 0);
            REQUIRE(C.rows() == 1);
            REQUIRE(C.cols() == 3);

            // C(:,1)
            CHECK(C(0, 0) == Approx(0.873121));

            // C(:,2)
            CHECK(C(0, 1) == Approx(0));

            // C(:,3)
            CHECK(C(0, 2) == Approx(0));
        }

        REQUIRE(SR.size() > 0);
        REQUIRE(SR.rows() == 1);
        REQUIRE(SR.cols() == 1);

        // SR(:,1)
        CHECK(SR(0, 0) == Approx(50));
    }
    REQUIRE(h.size() > 0);
    REQUIRE(h.rows() == 1);
    REQUIRE(h.cols() == 1);

    // h(:,1)
    CHECK(h(0, 0) == Approx(10256.3));
}
