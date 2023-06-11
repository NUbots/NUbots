/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2020 NUbots <nubots@nubots.net>
 */

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <fmt/format.h>

#include "utility/motion/splines/Combination.hpp"
#include "utility/motion/splines/Polynom.hpp"
#include "utility/motion/splines/SmoothSpline.hpp"

// Stores values of n, k and n choose k.
std::vector<std::vector<long int>> C = {{3, 0, 1},
                                        {13, 5, 1287},
                                        {39, 38, 39},
                                        {36, 7, 8347680},
                                        {46, 21, 6943526580276},
                                        {36, 22, 3796297200},
                                        {37, 32, 435897},
                                        {1, 1, 1},
                                        {36, 8, 30260340},
                                        {20, 6, 38760},
                                        {23, 15, 490314},
                                        {42, 11, 4280561376},
                                        {21, 3, 1330},
                                        {40, 16, 62852101650},
                                        {15, 0, 1},
                                        {50, 10, 10272278170},
                                        {42, 41, 42},
                                        {13, 11, 78},
                                        {28, 16, 30421755},
                                        {19, 15, 3876},
                                        {42, 28, 52860229080},
                                        {47, 17, 2741188875414},
                                        {20, 13, 77520},
                                        {3, 1, 3},
                                        {8, 2, 28},
                                        {12, 9, 220},
                                        {46, 36, 4076350421},
                                        {13, 11, 78},
                                        {24, 14, 1961256},
                                        {41, 26, 63432274896},
                                        {48, 9, 1677106640},
                                        {27, 0, 1},
                                        {26, 12, 9657700},
                                        {27, 7, 888030},
                                        {48, 11, 22595200368},
                                        {13, 5, 1287},
                                        {0, 0, 1},
                                        {40, 15, 40225345056},
                                        {44, 24, 1761039350070},
                                        {42, 12, 11058116888},
                                        {41, 33, 95548245},
                                        {3, 3, 1},
                                        {2, 2, 1},
                                        {36, 7, 8347680},
                                        {42, 30, 11058116888},
                                        {31, 21, 44352165},
                                        {12, 10, 66},
                                        {1, 0, 1},
                                        {11, 4, 330},
                                        {1, 1, 1},
                                        {33, 2, 528},
                                        {22, 5, 26334},
                                        {43, 21, 1052049481860},
                                        {13, 6, 1716},
                                        {9, 6, 84},
                                        {41, 21, 269128937220},
                                        {6, 1, 6},
                                        {7, 3, 35},
                                        {4, 3, 4},
                                        {24, 24, 1},
                                        {11, 2, 55},
                                        {23, 23, 1},
                                        {35, 18, 4537567650},
                                        {6, 4, 15},
                                        {27, 1, 27},
                                        {6, 2, 15},
                                        {21, 16, 20349},
                                        {24, 12, 2704156},
                                        {38, 32, 2760681},
                                        {46, 16, 991493848554},
                                        {13, 1, 13},
                                        {11, 0, 1},
                                        {28, 26, 378},
                                        {44, 5, 1086008},
                                        {21, 20, 21},
                                        {39, 23, 37711260990},
                                        {27, 6, 296010},
                                        {12, 8, 495},
                                        {22, 18, 7315},
                                        {27, 2, 351},
                                        {9, 9, 1},
                                        {9, 6, 84},
                                        {10, 7, 120},
                                        {25, 18, 480700},
                                        {22, 3, 1540},
                                        {48, 2, 1128},
                                        {50, 20, 47129212243960},
                                        {37, 21, 12875774670},
                                        {24, 6, 134596},
                                        {15, 1, 15},
                                        {39, 21, 62359143990},
                                        {44, 20, 1761039350070},
                                        {21, 21, 1},
                                        {14, 11, 364},
                                        {43, 0, 1},
                                        {1, 0, 1},
                                        {39, 21, 62359143990}};

// Test that the combination class works
// https://en.wikipedia.org/wiki/Combination
TEST_CASE("Test Combination", "[utility][motion][splines][Combination]") {
    utility::motion::splines::Combination comb;

    // Loop over all our test values and see if Combination gives the correct result for n choose k
    for (size_t i = 0; i < C.size(); i++) {
        const int& n   = C[i][0];
        const int& k   = C[i][1];
        const int& nCk = C[i][2];
        int combResult = comb.binomialCoefficient(k, n);
        INFO(n << " choose " << k << " = " << nCk << ", got " << combResult);
        REQUIRE(combResult == nCk);
    }
}

// Each vector represents the polynomial coefficients, the x value, and the y, y', y'', y''' values
std::vector<std::vector<long int>> P = {{3, 3, 2, 2, 0, 5, 11, 529985, 236357, 84352, 22584},
                                        {2, 3, 2, 3, 3, 1, 20, 6897261, 1698523, 334646, 49452},
                                        {1, 3, 2, 1, 5, 5, 2, 115, 209, 330, 396},
                                        {2, 2, 0, 2, 3, 2, 9, 131411, 71481, 31108, 10152},
                                        {2, 1, 2, 2, 1, 2, 12, 522158, 215185, 70996, 17580},
                                        {1, 2, 0, 2, 1, 0, 13, 428766, 160434, 48000, 10764},
                                        {2, 3, 1, 3, 1, 5, 13, 830991, 312560, 94048, 21222},
                                        {2, 3, 2, 0, 4, 2, 7, 41533, 28424, 15568, 6396},
                                        {3, 0, 2, 2, 4, 1, 12, 750289, 311956, 103828, 25932},
                                        {2, 0, 0, 0, 5, 5, 2, 79, 165, 320, 480},
                                        {0, 2, 2, 0, 0, 5, 5, 1505, 1150, 660, 252},
                                        {2, 3, 1, 1, 5, 5, 5, 8305, 7840, 5932, 3366},
                                        {3, 3, 0, 3, 1, 1, 10, 330311, 162061, 63606, 18720},
                                        {3, 1, 0, 1, 5, 3, 13, 1142677, 437234, 133850, 30732},
                                        {0, 0, 1, 3, 1, 3, 11, 1708, 430, 72, 6},
                                        {0, 0, 0, 0, 0, 4, 13, 4, 0, 0, 0},
                                        {2, 1, 3, 0, 3, 1, 19, 5103154, 1333898, 279034, 43794},
                                        {0, 1, 0, 2, 3, 4, 6, 1390, 891, 436, 144},
                                        {3, 2, 3, 1, 5, 3, 16, 3289427, 1018149, 252194, 46866},
                                        {2, 3, 3, 2, 5, 5, 10, 233255, 112945, 43784, 12738},
                                        {2, 0, 3, 3, 5, 4, 9, 120577, 66398, 29328, 9738},
                                        {3, 1, 0, 1, 1, 1, 13, 1142623, 437230, 133850, 30732},
                                        {2, 3, 3, 3, 5, 1, 2, 159, 309, 506, 642},
                                        {1, 3, 2, 3, 5, 0, 12, 314988, 125357, 39894, 9516},
                                        {2, 2, 1, 2, 3, 3, 16, 2232883, 688963, 170084, 31494},
                                        {2, 3, 3, 3, 0, 3, 20, 6905203, 1699720, 334766, 49458},
                                        {1, 0, 0, 3, 3, 4, 12, 249304, 103755, 34566, 8640},
                                        {3, 3, 1, 1, 4, 4, 5, 11424, 10964, 8432, 4866},
                                        {0, 3, 0, 1, 2, 2, 5, 1912, 1512, 902, 360},
                                        {2, 0, 0, 1, 0, 1, 2, 69, 164, 322, 480},
                                        {3, 2, 2, 2, 3, 0, 7, 56028, 39084, 21844, 9168},
                                        {3, 3, 3, 0, 2, 1, 11, 531092, 236678, 84414, 22590},
                                        {3, 0, 2, 1, 5, 0, 0, 0, 5, 2, 12},
                                        {3, 3, 3, 2, 3, 1, 10, 333231, 162943, 63784, 18738},
                                        {1, 3, 0, 1, 2, 4, 8, 45140, 26642, 12546, 4416},
                                        {3, 1, 0, 3, 0, 3, 15, 2329428, 772965, 205206, 40860},
                                        {2, 2, 1, 2, 1, 5, 12, 541169, 221665, 72652, 17862},
                                        {3, 3, 0, 2, 1, 3, 1, 12, 32, 100, 252},
                                        {0, 3, 2, 0, 5, 2, 20, 496102, 98405, 14640, 1452},
                                        {1, 3, 2, 2, 5, 3, 4, 1975, 2165, 1908, 1260},
                                        {0, 0, 1, 2, 1, 5, 8, 653, 225, 52, 6},
                                        {0, 2, 0, 1, 3, 2, 3, 182, 225, 218, 144},
                                        {0, 2, 0, 3, 0, 4, 4, 564, 536, 390, 192},
                                        {1, 1, 3, 2, 2, 3, 8, 38547, 23138, 11156, 4050},
                                        {0, 1, 3, 3, 0, 3, 8, 5827, 2672, 918, 210},
                                        {2, 3, 0, 0, 1, 3, 9, 137793, 74359, 32076, 10368},
                                        {1, 2, 1, 2, 5, 1, 3, 466, 665, 778, 690},
                                        {0, 1, 2, 1, 3, 3, 10, 12133, 4623, 1322, 252},
                                        {3, 2, 2, 0, 4, 3, 11, 515144, 230993, 82896, 22320},
                                        {3, 2, 1, 2, 5, 5, 12, 790049, 325349, 107212, 26502}};

// Given a polynomial, test that the polynomial class calculates the first, second and third derivatives correctly
TEST_CASE("Test Polynom", "[utility][motion][splines][Polynom]") {
    // Loop for each test case
    for (size_t i = 0; i < P.size(); i++) {
        // Get the coefficients for the test polynomial and the true differential values for the given x
        std::vector<long int> coefs = {P[i][5], P[i][4], P[i][3], P[i][2], P[i][1], P[i][0]};
        Eigen::Vector4i result_true(P[i][7], P[i][8], P[i][9], P[i][10]);
        int x = P[i][6];

        // Create the polynomial from test coefficients and get the computed differential values for the given x
        utility::motion::splines::Polynom poly(coefs);
        Eigen::Matrix<int long, 4, 1> result_calc(poly.pos(x), poly.vel(x), poly.acc(x), poly.jerk(x));

        // Log the polynomial, expected and computed values
        INFO("p(x) = " << coefs[0] << "x^5 + " << coefs[1] << "x^4 + " << coefs[2] << "x^3 + " << coefs[3] << "x^2 + "
                       << coefs[4] << "x + " << coefs[5] << ".");
        INFO("Pos: GOT " << result_true[0] << ", REQUIRED " << result_calc[0]);
        INFO("Vel: GOT " << result_true[1] << ", REQUIRED " << result_calc[1]);
        INFO("Acc: GOT " << result_true[2] << ", REQUIRED " << result_calc[2]);
        INFO("Jerk: GOT " << result_true[3] << ", REQUIRED " << result_calc[3]);

        // Test that we got the correct values
        REQUIRE(result_true[0] == result_calc[0]);
        REQUIRE(result_true[1] == result_calc[1]);
        REQUIRE(result_true[2] == result_calc[2]);
        REQUIRE(result_true[3] == result_calc[3]);
    }
}

const float ERROR = 1e-6f;  // Error for testing floats

// Test the smooth spline. Given a set of points (with first and second derivatives), the smooth spline will create
// piecewise polynomials. We will test that each polynomial satisfies the original contraints given, which are the
// points.
TEST_CASE("Test Smooth Spline", "[utility][motion][splines][SmoothSpline]") {
    utility::motion::splines::SmoothSpline<double> spline;

    // Generate random splines and test that the contraints hold
    for (int j = 0; j < 100; j++) {
        spline.reset();

        // Get a random number of points for this spline
        size_t noPoints = rand() % 6;
        std::vector<Eigen::Vector4d> points;

        // This value will keep track of the last t value - we want them to be consecutive, so we will check against
        // this. Start it negative since the generated t values will not be negative
        double point_t = -1.0;

        // Add our points to the spline
        for (size_t i = 0; i < noPoints; i++) {
            Eigen::Vector4d point = Eigen::Vector4d::Random();  // random 4d array of numbers between -1 and 1
            point *= 10;                                        // values are between -10 and 10
            point[0] = std::abs(point[0]);

            // If the last t and this t are not consecutive, add them together... plus 1 incase point_t is 0
            point[0] = (point[0] < point_t) ? (point[0] + point_t + 1) : point[0];

            spline.addPoint(point[0], point[1], point[2], point[3]);
            points.push_back(point);

            point_t = point[0];  // update the previous t value
        }

        // If there were not enough points to create any polynomials, make sure no splines were created and then jump to
        // the next iteration
        if (noPoints < 2) {
            if (spline.size() == 0) {  // No splines were made, continue to next iteration
                continue;
            }
            else {  // There were splines made - should not happen!
                INFO("Incorrect size. Expected 0, got " << spline.size() << ".");
                REQUIRE(false);
            }
        }

        // Ensure it has computed the correct number of splines
        if (spline.size() != (noPoints - 1)) {
            INFO("Incorrect size. Expected " << (noPoints - 1) << ", got " << spline.size() << ".");
            REQUIRE(false);
        }

        // For each spline, check contraints
        for (size_t i = 0; i < spline.size(); i++) {
            // This gives us the coefficients for the spline, a_0 to a_5
            std::vector<double> splineResult = spline.part(i).polynom.getCoefs();

            // Check contraints for first endpoint and then the second endpoint in the polynomial
            for (int k = 0; k < 2; k++) {

                // We need to check what this spline actually gives for x, and the first and second derivatives
                // of x, given an input of t. We will then check they match the expected values.
                double x   = 0.0;
                double xd  = 0.0;
                double xdd = 0.0;

                // Each spline is not made using the given t values. Each spline starts at t = 0 and the next point is
                // defined as t = second-first
                double t = ((k == 0) ? 0.0 : points[i + 1][0] - points[i][0]);

                // Loop over the coefficients, add to the calculations for each coefficient.
                // Position: x += a_j t^j
                // Velocity: xd = j * a_j * t^(j-1)
                // Acceleration: xdd = j * (j-1) * a_j * t^(j-2)
                // If j is less than 0, don't add anything since our derivative has become 0 for that term.
                for (int j = 0; j < int(splineResult.size()); j++) {
                    x += splineResult[j] * pow(t, j);
                    xd += (((j - 1) >= 0) ? j * splineResult[j] * pow(t, j - 1) : 0.0);
                    xdd += (((j - 2) >= 0) ? j * (j - 1) * splineResult[j] * pow(t, j - 2) : 0.0);
                }

                // Log calculated and expected values of each contraint, and the spline coefficients a_0 to a_5
                INFO(fmt::format("{} points. t = {}.", noPoints, t));
                INFO(fmt::format("For x, expected {}, got {}", points[i + k][1], x));
                INFO(fmt::format("For x', expected {}, got {}", points[i + k][2], x));
                INFO(fmt::format("For x'', expected {}, got {}", points[i + k][3], x));
                INFO(fmt::format("f(t) = {} + {}t + {}t^2 + {}t^3 + {}t^4 + {}t^5",
                                 splineResult[0],
                                 splineResult[1],
                                 splineResult[2],
                                 splineResult[3],
                                 splineResult[4],
                                 splineResult[5]));

                // Check that the contraints hold - do the computed values from the spline match the given random values
                // Check the difference is within a small error since we are using floating point values
                REQUIRE(abs(x - points[i + k][1]) < ERROR);
                REQUIRE(abs(xd - points[i + k][2]) < ERROR);
                REQUIRE(abs(xdd - points[i + k][3]) < ERROR);
            }
        }
    }
}
