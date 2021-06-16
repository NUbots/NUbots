/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef DARWIN_FSR_HPP
#define DARWIN_FSR_HPP

#include "DarwinDevice.hpp"

namespace Darwin {
    /**
     * @brief This represents, and gives access to the darwin's two Force Sensitive Resistor boards
     *
     * @author Trent Houliston
     */
    class FSR : public DarwinDevice {

    public:
        /**
         * @brief This enum holds the addresses of the various bytes in the FSR (Force Sensitive Resistors in the feet).
         *
         * @details
         *  for additional details see
         * http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
         */
        enum Address {
            MODEL_NUMBER_L         = 0,
            MODEL_NUMBER_H         = 1,
            VERSION                = 2,
            ID                     = 3,
            BAUD_RATE              = 4,
            RETURN_DELAY_TIME      = 5,
            RETURN_LEVEL           = 16,
            OPERATING_MODE         = 19,
            LED                    = 25,
            FSR1_L                 = 26,
            FSR1_H                 = 27,
            FSR2_L                 = 28,
            FSR2_H                 = 29,
            FSR3_L                 = 30,
            FSR3_H                 = 31,
            FSR4_L                 = 32,
            FSR4_H                 = 33,
            FSR_X                  = 34,
            FSR_Y                  = 35,
            PRESENT_VOLTAGE        = 42,
            REGISTERED_INSTRUCTION = 44,
            LOCK                   = 47
        };

        FSR(UART& coms, int id);
    };
}  // namespace Darwin

#endif
