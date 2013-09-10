/*
 * This file is part of DarwinMotionManager.
 *
 * DarwinMotionManager is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * DarwinMotionManager is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinMotionManager.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_PLATFORM_DARWIN_MOTIONMANAGER_H
#define MODULES_PLATFORM_DARWIN_MOTIONMANAGER_H

#include <list>
#include <thread>
#include <NUClear.h>
#include "messages/ServoWaypoint.h"

namespace modules {
namespace Platform {
namespace Darwin {

    class MotionManager : public NUClear::Reactor {
    private:
        struct Motion {
            NUClear::clock::time_point start;
            NUClear::clock::time_point end;
            float position;
            float gain;
            bool executed = false;
        };

        std::mutex waypointMutex;
        std::list<Motion> waypoints[20];

        void queueEnd(size_t queue);
        void allQueueEnd();
    public:
        explicit MotionManager(NUClear::PowerPlant* plant);
    };
}
}
}
#endif

