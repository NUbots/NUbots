
/*
 * This file is part of Darwin MotionManager.
 *
 * Darwin MotionManager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin MotionManager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin MotionManager.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_MOTION_WALKCOMMAND_H
#define MESSAGES_MOTION_WALKCOMMAND_H

#include <nuclear>

namespace messages {
    namespace motion {

        /**
         * TODO document
         *
         * @author Trent Houliston
         * @author Brendan Annable
         */
        struct WalkCommand {
                arma::vec2 velocity; // in m/s
                float rotationalSpeed; // in rads/s
        };
        
    }  // motion
}  // messages

#endif  // MESSAGES_MOTION_WALKCOMMAND_H