/*
 * This file is part of Darwin Motion Manager.
 *
 * Darwin Motion Manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Motion Manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Motion Manager.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
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

    /**
     * TODO document
     * 
     * @author Trent Houliston
     */
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

