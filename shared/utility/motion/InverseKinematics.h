/*
 * This file is part of InverseKinematics.
 *
 * InverseKinematics is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * InverseKinematics is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with InverseKinematics.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_INVERSEKINEMATICS_H
#define UTILITY_MOTION_INVERSEKINEMATICS_H

#include <vector>
#include <armadillo>

#include "messages/input/ServoID.h"

namespace utility {
namespace motion {
namespace kinematics {

		inline bool isInside(float t, float min, float max) {
			return min <= max ? t >= min && t <= max : t >= min || t <= max;
		}

		inline float limit(float t, float min, float max) {
			return t < min ? min : t > max ? max : t;
		}

		std::vector<std::pair<messages::input::ServoID, float>> calculateLegJoints(arma::mat44 target, bool isLeft);

} // kinematics
}  // motion
}  // utility

#endif  // UTILITY_MOTION_INVERSEKINEMATICS_H