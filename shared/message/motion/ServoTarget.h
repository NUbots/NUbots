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

#ifndef MESSAGES_MOTION_SERVOTARGET_H
#define MESSAGES_MOTION_SERVOTARGET_H

#include <nuclear>
#include "messages/input/ServoID.h"

namespace messages {
    namespace motion {

        /**
         * TODO document
         *
         * @author Trent Houliston
         */
        struct ServoTarget {
            NUClear::clock::time_point time;
            input::ServoID id;
            float position;
            float gain;
            float torque;
        };

    }  // motion
}  // messages

#endif  // MESSAGES_MOTION_SERVOTARGET_H