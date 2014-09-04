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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_ANGLE_H
#define UTILITY_MATH_ANGLE_H

#include <cmath>
#include <armadillo>

namespace utility {
namespace math {
    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    namespace angle {

        /**
         * Takes an angle in radians and normalizes it to be between -pi and pi
         *
         * @param value the angle in radians
         *
         * @return the angle between -pi and pi
         */
        inline double normalizeAngle(const double value) {

            double angle = std::fmod(value, 2 * M_PI);
            
            if (angle <= -M_PI)
                angle += M_PI * 2;

            if (angle > M_PI)
                angle -= 2 * M_PI;

            return angle;
        }

        /**
         * Calculates the difference between two angles between -pi and pi
         *
         * @param
         */
        inline double difference(const double a, const double b) {

            return M_PI - std::fabs(std::fmod(std::fabs(a - b), 2 * M_PI) - M_PI);
        }

        inline double vectorToBearing(arma::vec2 dirVec) {
            return std::atan2(dirVec(1), dirVec(0));
        }

        inline arma::vec2 bearingToUnitVector(double angle) {
            return arma::vec2({std::cos(angle), std::sin(angle)});
        }
    }
}
}
#endif
