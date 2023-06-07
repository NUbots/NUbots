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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_BALANCE_HPP
#define UTILITY_MOTION_BALANCE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <yaml-cpp/yaml.h>

#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/math/matrix/transform.hpp"

namespace utility::motion {

    class Balancer {
    private:
        // Config
        double rotationPGain = 0;
        double rotationIGain = 0;
        double rotationDGain = 0;

        double translationPGainX = 0;
        double translationPGainY = 0;
        double translationPGainZ = 0;

        double translationDGainX = 0;
        double translationDGainY = 0;
        double translationDGainZ = 0;

        double ankleRotationScale = 0;
        double hipRotationScale   = 0;

        // State
        double dPitch    = 0;
        double dRoll     = 0;
        double lastPitch = 0;
        double lastRoll  = 0;

        Eigen::Quaternion<double> lastErrorQuaternion;
        NUClear::clock::time_point lastBalanceTime;

    public:
        Balancer() = default;
        void configure(const YAML::Node& config);
        void balance(const message::actuation::KinematicsModel& model,
                     Eigen::Isometry3d& footToTorso,
                     const utility::input::LimbID& leg,
                     const message::input::Sensors& sensors);
    };
}  // namespace utility::motion

#endif
