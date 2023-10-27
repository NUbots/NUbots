/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef UTILITY_MOTION_SPLINES_FOOTSTEP_HPP
#define UTILITY_MOTION_SPLINES_FOOTSTEP_HPP

#include <Eigen/Core>

namespace utility::motion::splines {

    /**
     * Footstep
     *
     * Manage humanoid footstep generation and state
     */
    class Footstep {
    public:
        Footstep() = delete;
        /**
         * Initialization with lateral foot distance and support foot
         */
        Footstep(const float& foot_distance, const bool& is_left_support_foot = true);

        /**
         * Set the lateral foot distance parameters
         */
        constexpr void set_foot_distance(const float& foot_distance_) {
            foot_distance = foot_distance_;
        }
        [[nodiscard]] constexpr float getFootDistance() const {
            return foot_distance;
        }

        /**
         * Reset to neutral position the current step (not the integrated odometry)
         */
        void reset(const bool& is_left_support_foot);

        // Reset odometry
        void resetInWorld(const bool& is_left_support_foot);

        /**
         * Current support foot
         */
        [[nodiscard]] constexpr bool is_left_support() const {
            return is_left_support_foot;
        }

        /**
         * Starting position of current flying foot in support foot frame
         */
        [[nodiscard]] const Eigen::Vector3f& get_last() const {
            return support_to_last;
        }

        /**
         * Target pose of current flying foot in support foot frame
         */
        [[nodiscard]] const Eigen::Vector3f& get_next() const {
            return support_to_next;
        }

        /**
         * Returns the odometry change of the current step.
         */
        [[nodiscard]] const Eigen::Vector3f& getOdom() const;

        /**
         * Left and right, current or next pose of foot in world initial frame
         */
        [[nodiscard]] const Eigen::Vector3f& getLeft() const {
            return left_in_world;
        }
        [[nodiscard]] const Eigen::Vector3f& getRight() const {
            return right_in_world;
        }

        /**
         * Set the target pose of current support foot during next support phase and update support foot.
         * The target foot pose diff is given with respect to next support foot pose (current flying foot
         * target).
         */
        void stepFromSupport(const Eigen::Vector3f& diff);

        /**
         * Set target pose of current support foot using diff orders.
         * Zero vector means in place walking.
         * Special handle of lateral and turn step to avoid foot collision.
         */
        void step_from_orders(const Eigen::Vector3f& diff);

    private:
        /**
         * Static lateral distance between the feet
         */
        float foot_distance;

        /**
         * Current support foot
         * (left or right)
         */
        bool is_left_support_foot;

        /**
         * Pose diff [dx, dy, dtheta] from support foot to flying foot last and next position
         */
        Eigen::Vector3f support_to_last = Eigen::Vector3f::Zero();
        Eigen::Vector3f support_to_next = Eigen::Vector3f::Zero();

        /**
         * Pose integration of left and right foot in initial frame.
         * Set at "future" state taking into account next expected fot pose.
         */
        Eigen::Vector3f left_in_world  = Eigen::Vector3f::Zero();
        Eigen::Vector3f right_in_world = Eigen::Vector3f::Zero();
    };

}  // namespace utility::motion::splines
#endif
