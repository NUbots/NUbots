/*
 * MIT License
 *
 * Copyright (c) 2015 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
        float hipRotationScale   = 0;

        // State
        float dPitch    = 0;
        float dRoll     = 0;
        float lastPitch = 0;
        float lastRoll  = 0;

        Eigen::Quaternion<float> lastErrorQuaternion;
        NUClear::clock::time_point lastBalanceTime;

    public:
        Balancer() = default;
        void configure(const YAML::Node& config);
        void balance(const message::actuation::KinematicsModel& model,
                     Eigen::Isometry3f& footToTorso,
                     const utility::input::LimbID& leg,
                     const message::input::Sensors& sensors);
    };
}  // namespace utility::motion

#endif
