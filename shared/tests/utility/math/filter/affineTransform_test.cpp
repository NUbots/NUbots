// #include <Eigen/Core>
// #include <catch.hpp>

// #include "iCanHazPolarCoordz.h"

// #include "utility/math/filter/gaussian.hpp"


// SCENARIO("affineTranform: Calling ICanHazPolarCoordz with mux = [1, 3]") {

//     int nx = 2;
//     Eigen::VectorXd mux(nx), muy;

//     Eigen::MatrixXd Sxx(nx, nx), Syy;

//     // mux - [2 x 1]:
//     mux << 1, 3;

//     // Sxx - [2 x 2]:
//     Sxx << 10, 0, 0, 1;

//     ICanHazPolarCoordz func;

//     WHEN("Calling affineTransform") {
//         utility::math::filter::gaussian::affineTransform(mux, Sxx, func, muy, Syy);

//         //--------------------------------------------------------------------------------
//         // Checks for muy
//         //--------------------------------------------------------------------------------
//         THEN("muy is not empty") {
//             REQUIRE(muy.size() > 0);

//             AND_THEN("muy has the right dimensions") {
//                 REQUIRE(muy.rows() == 2);
//                 REQUIRE(muy.cols() == 1);
//                 AND_THEN("muy is correct") {

//                     // muy(:,1)
//                     CHECK(muy(0, 0) == Approx(0.321750554397));
//                     CHECK(muy(1, 0) == Approx(3.16227766017));
//                 }
//             }
//         }

//         //--------------------------------------------------------------------------------
//         // Checks for Syy
//         //--------------------------------------------------------------------------------
//         THEN("Syy is not empty") {
//             REQUIRE(Syy.size() > 0);

//             AND_THEN("Syy has the right dimensions") {
//                 REQUIRE(Syy.rows() == 2);
//                 REQUIRE(Syy.cols() == 2);
//                 AND_THEN("Syy is correct") {

//                     // Syy(:,1)
//                     CHECK(Syy(0, 0) == Approx(-3.00171694492));
//                     CHECK(Syy(1, 0) == Approx(0));

//                     // Syy(:,2)
//                     CHECK(Syy(0, 1) == Approx(-3.12886418774));
//                     CHECK(Syy(1, 1) == Approx(-1.05839921328));
//                 }
//             }
//         }
//     }
// }


// SCENARIO("affineTranform: Calling ICanHazPolarCoordz with mux = [1, 1]") {
//     int nx = 2;
//     Eigen::VectorXd mux(nx), muy;

//     Eigen::MatrixXd Sxx(nx, nx), Syy;

//     // mux - [2 x 1]:
//     mux << 1, 1;

//     // Sxx - [2 x 2]:
//     Sxx << 1.5, 0, 0, 1.8;

//     ICanHazPolarCoordz func;

//     WHEN("Calling affineTransform") {
//         utility::math::filter::gaussian::affineTransform(mux, Sxx, func, muy, Syy);

//         //--------------------------------------------------------------------------------
//         // Checks for muy
//         //--------------------------------------------------------------------------------
//         THEN("muy is not empty") {
//             REQUIRE(muy.size() > 0);

//             AND_THEN("muy has the right dimensions") {
//                 REQUIRE(muy.rows() == 2);
//                 REQUIRE(muy.cols() == 1);
//                 AND_THEN("muy is correct") {

//                     // muy(:,1)
//                     CHECK(muy(0, 0) == Approx(0.785398163397));
//                     CHECK(muy(1, 0) == Approx(1.41421356237));
//                 }
//             }
//         }

//         //--------------------------------------------------------------------------------
//         // Checks for Syy
//         //--------------------------------------------------------------------------------
//         THEN("Syy is not empty") {
//             REQUIRE(Syy.size() > 0);

//             AND_THEN("Syy has the right dimensions") {
//                 REQUIRE(Syy.rows() == 2);
//                 REQUIRE(Syy.cols() == 2);
//                 AND_THEN("Syy is correct") {

//                     // Syy(:,1)
//                     CHECK(Syy(0, 0) == Approx(-1.17166745172));
//                     CHECK(Syy(1, 0) == Approx(0));

//                     // Syy(:,2)
//                     CHECK(Syy(0, 1) == Approx(0.298734812658));
//                     CHECK(Syy(1, 1) == Approx(-1.63271476741));
//                 }
//             }
//         }
//     }
// }
