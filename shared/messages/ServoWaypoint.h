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

#ifndef MESSAGES_SERVOWAYPOINTS_H
#define MESSAGES_SERVOWAYPOINTS_H

#include <NUClear.h>
#include "DarwinServoCommand.h"

namespace messages {

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    template<enum DarwinSensors::Servo::ID>
    struct ServoWaypointsComplete {};

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct AllServoWaypointsComplete {};

    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    struct ServoWaypoint {
        NUClear::clock::time_point time;
        DarwinSensors::Servo::ID id;
        float position;
        float gain;
    };
};

#endif