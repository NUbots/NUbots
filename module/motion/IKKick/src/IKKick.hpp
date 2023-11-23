/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef MODULES_MOTION_IKKICK_HPP
#define MODULES_MOTION_IKKICK_HPP

#include <nuclear>

#include "IKKickControllers.hpp"

#include "message/motion/KickCommand.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/motion/Balance.hpp"

namespace module::motion {

    class IKKick : public NUClear::Reactor {

    private:
        // ID of support foot
        utility::input::LimbID supportFoot;
        // NEED the vector from the point on the surface of the ball where we want to kick to the front of the kick
        // foot which is rightFootFront KickPlanner has to add the radius of the all to get the location of the
        // centre of the ball point position of ball
        Eigen::Vector3d ballPosition;
        // direction we want to kick the ball
        Eigen::Vector3d goalPosition;

        /// Subsumption ID key to access motors
        const size_t subsumptionId;

        bool leftFootIsSupport;

        float foot_separation;

        float KICK_PRIORITY;
        float EXECUTION_PRIORITY;

        float gain_legs = 50;
        float torque    = 100;

        bool feedback_active;
        utility::motion::Balancer feedbackBalancer;

        KickBalancer balancer;
        Kicker kicker;

        void updatePriority(const float& priority);

        static constexpr size_t UPDATE_FREQUENCY = 90;

        ReactionHandle updater;

    public:
        /// @brief Called by the powerplant to build and setup the IKKick reactor.
        explicit IKKick(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::motion


#endif
