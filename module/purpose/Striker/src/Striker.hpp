/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_PURPOSE_STRIKER_HPP
#define MODULE_PURPOSE_STRIKER_HPP


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Striker : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Calls Tasks to play soccer normally for a striker
        void play();

        /// @brief Stores configuration values
        struct Config {
            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3f ready_position = Eigen::Vector3f::Zero();
            /// @brief Penalty defence position to walk to when defending against a penalty kick from the opposing team
            Eigen::Vector3f penalty_defence_position = Eigen::Vector3f::Zero();
            /// @brief Ready position to walk to
            Eigen::Isometry3d Hfr = Eigen::Isometry3d::Identity();
            /// @brief How far (m) the ball can be away from the centre of the field for it to be deemed as
            /// moved/touched by the other team during kickoff so we can start playing
            double ball_kickoff_outside_radius = 0.0;
            /// @brief x minimum bound on field to walk within
            double bounded_region_x_min = 0.0;
            /// @brief x maximum bound on field to walk within
            double bounded_region_x_max = 0.0;
            /// @brief y minimum bound on field to walk within
            double bounded_region_y_min = 0.0;
            /// @brief y maximum bound on field to walk within
            double bounded_region_y_max = 0.0;
        } cfg;


    public:
        /// @brief Called by the powerplant to build and setup the Striker reactor.
        explicit Striker(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_STRIKER_HPP
