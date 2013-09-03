/*
 * This file is part of NCursesReactor.
 *
 * NCursesReactor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * NCursesReactor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with NCursesReactor.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "ScriptTuner.h"

namespace modules {

    ScriptTuner::ScriptTuner(NUClear::PowerPlant* plant) : Reactor(plant) {

        /*
         *                      SCRIPT TUNER
         * Script   Script name
         * FRAME 1 2 3 4 5 6 7 8 9 10
         * [Run] [Step]
         * Duration: 1000
         *
         * U Head Pan               0.00 radians
         * U Head Tilt              0.00 radians
         * L Right Shoulder Pitch   0.00 radians
         * U Left Shoulder Pitch    0.00 radians
         * L Right Shoulder Roll    0.00 radians
         * L Left Shoulder Roll     0.00 radians
         * L Right Elbow            0.00 radians
         * L Left Elbow             0.00 radians
         * L Right Hip Yaw          0.00 radians
         * L Left Hip Yaw           0.00 radians
         * L Right Hip Roll         0.00 radians
         * L Left Hip Roll          0.00 radians
         * L Right Hip Pitch        0.00 radians
         * L Left Hip Pitch         0.00 radians
         * L Right Knee             0.00 radians
         * L Left Knee              0.00 radians
         * L Right Ankle Pitch      0.00 radians
         * L Left Ankle Pitch       0.00 radians
         * L Right Ankle Roll       0.00 radians
         * L Left Ankle Roll        0.00 radians
         */

    }
}
