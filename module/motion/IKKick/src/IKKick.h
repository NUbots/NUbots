/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_MOTION_IKKICK_H
#define MODULES_MOTION_IKKICK_H

#include <nuclear>

#include "IKKickControllers.h"
#include "message/motion/KickCommand.h"
#include "utility/input/LimbID.h"
#include "utility/motion/Balance.h"

namespace module {
namespace motion {

    class IKKick : public NUClear::Reactor {

    private:
        // ID of support foot
        utility::input::LimbID supportFoot;
        // NEED the vector from the point on the surface of the ball where we want to kick to the front of the kick foot
        // which is rightFootFront
        // KickPlanner has to add the radius of the all to get the location of the centre of the ball
        // point position of ball
        arma::vec3 ballPosition;
        // direction we want to kick the ball
        arma::vec3 goalPosition;

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
}  // namespace motion
}  // namespace module


#endif
