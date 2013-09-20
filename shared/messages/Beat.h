/*
 * This file is part of Beat Detector.
 *
 * Beat Detector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Beat Detector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Beat Detector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#ifndef MESSAGES_BEAT_H
#define	MESSAGES_BEAT_H

#include <NUClear.h>

namespace messages {
    struct Beat {
        NUClear::clock::time_point time;
        NUClear::clock::duration period;
    };
}

#endif  // MESSAGES_BEAT_H

