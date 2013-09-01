/*
 * This file is part of MotionManager.
 *
 * MotionManager is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MotionManager is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with MotionManager.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MESSAGES_SERVOWAYPOINTS_H
#define MESSAGES_SERVOWAYPOINTS_H

#include <NUClear.h>
#include "DarwinServoCommand.h"

namespace messages {

    template<enum DarwinSensors::Servo::ID>
    struct ServoWaypointsComplete {};

    struct AllServoWaypointsComplete {};

    struct ServoWaypoint {
        NUClear::clock::time_point time;
        DarwinSensors::Servo::ID id;
        float position;
        float gain;
    };
};

#endif