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
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/strategy/TeamMates.hpp"


namespace utility::strategy {

    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::strategy::TeamMates;

    struct Possession {
        enum Value { SELF, TEAMMATE, OPPONENT, NONE };
        Value value = Value::SELF;
    };


    std::vector<std::pair<Possession, double>> get_sorted_bots(const Ball& ball,
                                                               const TeamMates& teammates,
                                                               const Field& field,
                                                               const Sensors& sensors) {
        // Create empty list.
        std::vector<std::pair<Possession, double>> robots{};

        // Transform ball position to field coordinates.
        // 'ball.rBWw' is ball position in world coordinates.
        // 'field.Hfw' transforms from world to field coordinates.
        // Multiplying these gives 'rBFf', which is the ball's position in field coordinates.
        Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

        // Find self distance to ball.
        // 'sensors.Hrw' transforms world to robot.
        Eigen::Vector3d rBRr         = sensors.Hrw * ball.rBWw;
        double self_distance_to_ball = rBRr.norm();
        robots.push_back({Possession{Possession::SELF}, self_distance_to_ball});

        // Loop through each teammate,
        // subtract ball position (rBFf) from teammates position (rRFf) to get vector between both.
        for (const auto& mate : teammates.teammates) {
            Eigen::Vector3d rRFf    = mate.rRFf;
            double distance_to_ball = (rRFf - rBFf).norm();
            robots.push_back({Possession{Possession::TEAMMATE}, distance_to_ball});
        }

        // Robots are sorted by distance to the ball.
        // Compare distance smallest to largest.
        std::sort(robots.begin(), robots.end(), [](const auto& a, const auto& b) {
            //
            return a.second < b.second;
        });

        // Sorted list of robots is returned.
        return robots;
    }


    Possession get_possession(const Ball& ball,
                              const TeamMates& teammates,
                              const Field& field,
                              const Sensors& sensors,
                              double threshold) {

        // Function determines who has possession based on proximity and a threshold distance.
        // First calls 'get_sorted_bots()' to get the list of robots sorted by distance to ball.
        // If no robot close, return NONE.
        // If closest robot too far, return NONE.
        // Otherwise check if SELF, TEAMMATE, or OPPONENT.
        auto sorted_robots = get_sorted_bots(ball, teammates, field, sensors);

        if (sorted_robots.empty()) {
            return Possession{Possession::NONE};
        }

        auto& closest_bot = sorted_robots[0];

        if (closest_bot.second > threshold) {
            return Possession{Possession::NONE};
        }
        else if (closest_bot.first.value == Possession::SELF) {
            return Possession{Possession::SELF};
        }
        else if (closest_bot.first.value == Possession::TEAMMATE) {
            return Possession{Possession::TEAMMATE};
        }
        else {
            return Possession{Possession::OPPONENT};
        }
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
