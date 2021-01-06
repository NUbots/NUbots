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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <catch.hpp>
// #include <cmath>

#include "utility/motion/splines/Combination.hpp"
#include "utility/motion/splines/Polynom.hpp"
#include "utility/motion/splines/SmoothSpline.hpp"
#include "utility/motion/splines/Spline.hpp"

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

TEST_CASE("Test Combination", "[utility][motion][splines][Combination]") {
    utility::motion::splines::Combination comb;

    // Loop over all our test values and see if Combination gives the correct result
    for (size_t i = 0; i < C.size(); i++) {
        int n          = C[i][0];
        int k          = C[i][1];
        int nCk        = C[i][2];
        int combResult = comb.binomialCoefficient(k, n);
        INFO(n << " choose " << k << " = " << nCk << ", got " << combResult);
        REQUIRE(combResult == nCk);
    }
}

// Each vector represents
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


TEST_CASE("Test Polynom", "[utility][motion][splines][Polynom]") {
    for (size_t i = 0; i < P.size(); i++) {
        int one       = P[i][5];
        int two       = P[i][4];
        int three     = P[i][3];
        int four      = P[i][2];
        int five      = P[i][1];
        int six       = P[i][0];
        int x         = P[i][6];
        long int pos  = P[i][7];
        long int vel  = P[i][8];
        long int acc  = P[i][9];
        long int jerk = P[i][10];

        std::vector<int> coefs = {one, two, three, four, five, six};
        utility::motion::splines::Polynom poly(coefs);

        int long poly_pos  = poly.pos(x);
        int long poly_vel  = poly.vel(x);
        int long poly_acc  = poly.acc(x);
        int long poly_jerk = poly.jerk(x);

        INFO("p(x) = " << one << "x^5 + " << two << "x^4 + " << three << "x^3 + " << four << "x^2 + " << five << "x + "
                       << six << ".");

        INFO("Pos: GOT " << pos << ", REQUIRED " << poly_pos);
        INFO("Vel: GOT " << vel << ", REQUIRED " << poly_vel);
        INFO("Acc: GOT " << acc << ", REQUIRED " << poly_acc);
        INFO("Jerk: GOT " << jerk << ", REQUIRED " << poly_jerk);

        REQUIRE(pos == poly_pos);
        REQUIRE(vel == poly_vel);
        REQUIRE(acc == poly_acc);
        REQUIRE(jerk == poly_jerk);
    }
}

std::vector<float> times                = {1.0f, 2.0f, 3.0f};
std::vector<float> position             = {1.0f, 2.0f, 1.0f};
std::vector<float> velocity             = {-2.0f, 0.0f, -2.0f};
std::vector<float> acceleration         = {0.0f, 0.0f, 0.0f};
std::vector<std::vector<float>> splines = {{12.0f, -91.0f, 266.0f, -372.0f, 248.0f, -62.0f},
                                           {-12.0f, 149.0f, -730.0f, 1764.0f, -2104.0f, 994.0f}};

TEST_CASE("Test Smooth Spline", "[utility][motion][splines][SmoothSpline]") {
    utility::motion::splines::SmoothSpline<float> spline();


    for (int i = 0; i < times.size(); i++) {
        spline.addPoint(times[i], position[i], velocity[i], acceleration[i]);
    }

    spline.computerSplines();

    if (spline.size() != splines.size()) {
        INFO("Incorrect size. Expected " << splines.size() << ", got " << spline.size() << ".");
        REQUIRE(false);
    }

    for (int i = 0; i < spline.size(); i++) {
        std::vector<Scalar> splineResult = spline.get(i).polynom.getCoefs();
        for (int j = 0; j < splineResult.size(); j++) {
            REQUIRE(splineResult[j] == splines[i][j]);
        }
    }
}
