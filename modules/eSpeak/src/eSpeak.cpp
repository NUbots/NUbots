/*
 * This file is part of eSpeakReactor.
 *
 * eSpeakReactor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * eSpeakReactor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with eSpeakReactor.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "eSpeak.h"

#include <cstdlib>

#include "Messages/Say.h"

namespace modules {

    eSpeak::eSpeak(NUClear::PowerPlant* plant) : Reactor(plant) {
        on<Trigger<Say>>([](const Say& message) {
            system("espeak -k6 -a1000 -ven '" + message + "'");
        });
    }
}
