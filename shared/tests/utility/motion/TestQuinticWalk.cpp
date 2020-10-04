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

#include "utility/motion/QuinticWalk/Combination.hpp"
#include "utility/motion/QuinticWalk/Footstep.hpp"
#include "utility/motion/QuinticWalk/Polynom.hpp"
#include "utility/motion/QuinticWalk/SmoothSpline.hpp"
#include "utility/motion/QuinticWalk/Spline.hpp"
#include "utility/motion/QuinticWalk/SplineContainer.hpp"
#include "utility/motion/QuinticWalk/TrajectoryUtils.hpp"
#include "utility/motion/QuinticWalk/WalkEngine.hpp"

std::vector<std::vector<long int>> C = {{3, 0, 1},
                                        {3, 0, 1},
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
                                        {1, 1, 1},
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
                                        {1, 0, 1},
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

TEST_CASE("Test Combination", "[utility][motion][QuinticWalk][Combination]") {
    utility::motion::quinticwalk::Combination comb;

    for (size_t i = 0; i < C.size(); i++) {
        int n          = C.at(i).at(0);
        int k          = C.at(i).at(1);
        int nCk        = C.at(i).at(2);
        int combResult = comb.binomialCoefficient(k, n);
        INFO(n << " choose " << k << " = " << nCk << ", got " << combResult);
        REQUIRE(combResult == nCk);
    }
}
