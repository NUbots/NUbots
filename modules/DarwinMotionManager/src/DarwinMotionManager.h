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

#ifndef MODULES_DARWINMOTIONMANAGER_H
#define MODULES_DARWINMOTIONMANAGER_H

#include <list>
#include <NUClear.h>
#include "messages/ServoWaypoint.h"

namespace modules {

    class DarwinMotionManager : public NUClear::Reactor {
    private:
        struct Motion {
            NUClear::clock::time_point start;
            NUClear::clock::time_point end;
            float position;
            float gain;
            bool executed = false;
        };

        std::list<Motion> waypoints[20];

        void queueEnd(size_t queue);
        void allQueueEnd();
    public:
        explicit DarwinMotionManager(NUClear::PowerPlant* plant);
    };
}
#endif

