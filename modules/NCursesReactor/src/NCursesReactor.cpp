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

#include "NCursesReactor.h"

#include <cursesm.h>

namespace modules {

    NCursesReactor::NCursesReactor(NUClear::PowerPlant* plant) : Reactor(plant) {

        // Get into curses mode

        // Set ourselves to be in raw mode

        // Setup our window



        // TODO this should be our global NCurses application instance
        // It should handle all input of data from NCurses and emit them
        // It should contain all the other windows in the system and use the F1-12 keys to swap between
        // The various windows
    }
}
