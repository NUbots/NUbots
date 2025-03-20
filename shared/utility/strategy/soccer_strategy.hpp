/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
#define UTILITY_STRATEGY_SOCCER_STRATEGY_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/input/RoboCup.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/TeamMates.hpp"


namespace utility::strategy {

    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::TeamMates;

    struct Possession {
        enum Value { SELF, TEAMMATE, OPPONENT, NONE };
        Value value = Value::SELF;
    };

    Possession get_possession(const Ball& ball, const TeamMates& teammates, const Field& field, double threshold) {
        // Do we have the ball?
        Eigen::Vector3d rBCc =
            ball.Hcw * ball.rBWw;  // cam 2 ball equals the world 2 cam transform times the world 2 ball
        rBCc.norm();
        if (rBCc.norm() < threshold) {  // if the cam 2 ball norm is less than the threshold than return the possession
                                        // of the current robot.
            return Possession{Possession::SELF};
        }

        // Do our teammates have the ball?
        Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;  // Get the ball relative to the field
        for (const auto& teammate : teammates.teammates)
        // Loop through the team mates
        {                                                 // for teammate in teammates.teammates
            Eigen::Vector3d rRBf = teammate.rRFf - rBFf;  // Eigen::Vector3d rRBf = mate.rRFf - rBFf;
            if (rRBf.norm()
                < threshold) {  // if rRBf.norm() < threshold then teammate has ball return Possession::TEAMMATE
                return Possession{Possession::TEAMMATE};
            }
        }

        // other?
        return Possession{Possession::NONE};
    }

}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
