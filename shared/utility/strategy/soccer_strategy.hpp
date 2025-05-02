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

    struct Robots {
        // Robot's distance to ball
        double distance_to_ball = 0.0;
        // Id of the robot
        uint id = 0;
        // Instance of Possession struct, indicating if the robot has the ball, is close, etc.
        Possession possession;
    };

    std::vector<Robots> get_sorted_bots(const Ball& ball, const TeamMates& teammates, const Field& field) {
        std::vector<Robots> robots;

        // Transforms the ball's position from world coordinates to field coordinates using field.Hfw.
        // rBWw is the ball's position in world coordinates.
        Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;

        // Looping through teammmates, for each teammate, a 'Robots' object is created.
        // Robot's position (mate.rRFf) is retrieved, and distance from robot to ball is calculated using norm.
        // Then is stored in 'distance_to_ball'
        // Each 'Robots' object is added to the robots vector.
        for (const auto& mate : teammates.teammates) {
            Robots robot;
            robot.id               = mate.id;
            Eigen::Vector3d rRFf   = mate.rRFf;
            robot.distance_to_ball = (rRFf - rBFf).norm();
            robots.push_back(robot);
        }

        // Robot's are sorted by distance to the ball, if there is a tie, they are sorted by robot id.
        std::sort(robots.begin(), robots.end(), [](const Robots& a, const Robots& b) {
            return (a.distance_to_ball < b.distance_to_ball)
                   || (a.distance_to_ball == b.distance_to_ball && a.id < b.id);
        });

        // Sorted list of robots is returned.
        return robots;
    }


    Possession get_possession(const Ball& ball, const TeamMates& teammates, const Field& field, double threshold) {

        // Function determines who has possession based on proximity and a threshold distance.
        // First calls 'get_sorted_bots()' to get the list of robots sotered by distance to ball.
        // If no robot's are found, it returns 'Possession::NONE'
        std::vector<Robots> sortedRobots = get_sorted_bots(ball, teammates, field);
        if (sortedRobots.empty()) {
            return Possession{Possession::NONE};
        }

        // rBCc transforms the ball's position from world coordinates to camera coordinates using ball.Hcw.
        // If the distance between the ball and robot is less than the threshold, it will assume the robot has possesion
        // 'Possession::SELF' is returned
        Eigen::Vector3d rBCc = ball.Hcw * ball.rBWw;

        if (rBCc.norm() < threshold) {
            return Possession{Possession::SELF};
        }

        // Function checks each teammate, calculates the distance from each teammate to the ball's position.
        // If there is a teammate within the threshold, 'Possession::TEAMMATE' is returned.
        Eigen::Vector3d rBFf = field.Hfw * ball.rBWw;
        for (const auto& teammate : teammates.teammates) {
            Eigen::Vector3d rRBf = teammate.rRFf - rBFf;
            if (rRBf.norm() < threshold) {
                return Possession{Possession::TEAMMATE};
            }
        }

        // If no robot or teammate id within the threshold, 'Possession::NONE' is returned.
        return Possession{Possession::NONE};
    }
}  // namespace utility::strategy

#endif  // UTILITY_STRATEGY_SOCCER_STRATEGY_HPP
