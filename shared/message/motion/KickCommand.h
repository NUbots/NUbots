/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_MOTION_KICKCOMMAND_H
#define MESSAGE_MOTION_KICKCOMMAND_H

#include <nuclear>
#include <armadillo>
#include "message/input/LimbID.h"

namespace message {
    namespace motion {

        /**
         * TODO document
         *
         * @author Trent Houliston
         * @author Brendan Annable
         */
        enum class KickCommandType {
            NORMAL,
            POWER
        };

        struct KickCommand {
            arma::vec3 target; // The point to kick
            arma::vec3 direction; // force is the magnitude
            KickCommandType kickCommandType = KickCommandType::NORMAL;

            KickCommand(const arma::vec3& target, const arma::vec3& direction, const KickCommandType& kickCommandType = KickCommandType::NORMAL)
            : target(target)
            , direction(direction)
            , kickCommandType(kickCommandType) {}
        };

        /**
         * TODO document
         *
         * @author Trent Houliston
         * @author Brendan Annable
         */
        struct KickScriptCommand {
            arma::vec3 direction; // Direction to kick with magnitude determining force
            message::input::LimbID leg; // Leg to kick with
        };


        struct KickPlannerConfig{
            float max_ball_distance;
            float kick_corridor_width;
            float seconds_not_seen_limit;
            float kick_forward_angle_limit;
        };

        struct KickFinished{
        };

        struct IKKickParams{
            float stand_height;
        };

    }  // motion
}  // message

#endif  // MESSAGE_MOTION_KICKCOMMAND_H
