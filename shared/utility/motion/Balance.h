/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_BALANCE_H
#define UTILITY_MOTION_BALANCE_H


#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"
#include "utility/motion/RobotModels.h"

#include "message/input/Sensors.h"
#include "message/input/LimbID.h"
#include "message/support/Configuration.h"
#include <yaml-cpp/yaml.h>

#include <nuclear>


namespace utility {
namespace motion {

    class Balancer {
    private:
        //Config
        float rotationPGain = 0;
        float rotationIGain = 0;
        float rotationDGain = 0;

        float translationPGainX = 0;
        float translationPGainY = 0;
        float translationPGainZ = 0;

        float translationDGainX = 0;
        float translationDGainY = 0;
        float translationDGainZ = 0;

        float ankleRotationScale = 0;
        float hipRotationScale = 0;

        //State
        float dPitch = 0;
        float dRoll = 0;
        float lastPitch = 0;
        float lastRoll = 0;

        utility::math::geometry::UnitQuaternion lastErrorQuaternion;
        NUClear::clock::time_point lastBalanceTime;
    public:
        void configure(const YAML::Node& config);
        void balance(utility::math::matrix::Transform3D& footToTorso, const message::input::LimbID& leg, const message::input::Sensors& sensors);
    };


}
}

#endif
