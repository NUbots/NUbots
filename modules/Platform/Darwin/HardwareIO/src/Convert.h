/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_PLATFORM_DARWIN_CONVERT_H
#define MODULES_PLATFORM_DARWIN_CONVERT_H

#include <NUClear.h>

namespace modules {
namespace Platform {
namespace Darwin {
    struct Convert {

        static float acceleronometer(const uint16_t value);
        static float gyroscope(const uint16_t value);
        static float voltage(const uint8_t value);
        static float fsrForce(const uint16_t value);
        static float fsrCentre(const bool left, const bool x, const uint8_t value);

        static std::tuple<uint8_t, uint8_t, uint8_t> colourLED(uint16_t value);
        static uint16_t colourLEDInverse(uint8_t r, uint8_t g, uint8_t b);

        static float gain(const uint8_t value);
        static uint8_t gainInverse(const float value);

        static float servoPosition(const uint8_t id, const uint16_t value);
        static uint16_t servoPositionInverse(const uint8_t id, const float value);

        static float speed(const uint8_t id, const uint16_t value);
        static uint16_t speedInverse(const uint8_t id, const float value);

        static float torqueLimit(const uint16_t value);

        static float load(const uint8_t id, const uint8_t value);

        static float temperature(const uint8_t value);
    };
}
}
}
#endif

