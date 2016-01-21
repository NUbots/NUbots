/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_RANSACGOALMODEL_H
#define MODULES_VISION_RANSACGOALMODEL_H

#include <nuclear>

#include <array>
#include "utility/math/geometry/Line.h"

namespace module {
namespace vision {

    class RansacGoalModel {
    public:
        utility::math::geometry::Line left;
        utility::math::geometry::Line right;

        static constexpr size_t REQUIRED_POINTS = 2;

        struct GoalSegment {
            arma::vec2 left;
            arma::vec2 right;
        };

        using DataPoint = GoalSegment;

        RansacGoalModel() {}

        bool regenerate(const std::array<DataPoint, REQUIRED_POINTS>& pts);

        double calculateError(const DataPoint& p) const;

        template <typename Iterator>
        void refineModel(Iterator& begin, Iterator& end, const double& threshold) {

            // Allows us to iterate through only the left states without copying
            struct LIt {
                Iterator state;
                LIt(Iterator state) : state(state) {}
                LIt operator++() { return ++state; }
                const arma::vec2& operator*() { return state->left; }
                bool operator!=(const LIt& other) { return state != other.state; }
            };

            // Allows us to iterate through only the right states without copying
            struct RIt {
                Iterator state;
                RIt(Iterator state) : state(state) {}
                RIt operator++() { return ++state; }
                const arma::vec2& operator*() { return state->right; }
                bool operator!=(const RIt& other) { return state != other.state; }
            };

            left.leastSquaresUpdate(LIt(begin), LIt(end), threshold);
            right.leastSquaresUpdate(RIt(begin), RIt(end), threshold);
        }

};

}
}


#endif