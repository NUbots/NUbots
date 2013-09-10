/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DarwinPlatform.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_DARWINPLATFORM_H
#define MODULES_DARWINPLATFORM_H

#include <NUClear.h>

#include "Darwin.h"

namespace modules {

    /**
     * This NUClear Reactor is responsible for reading in the data for the DarwinPlatform and emitting it to the rest
     * of the system
     * 
     * @author Trent Houliston
     */
    class DarwinPlatform : public NUClear::Reactor {
    private:
        /// @brief Our internal darwin class that is used for interacting with the hardware
        Darwin::Darwin darwin;

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit DarwinPlatform(NUClear::PowerPlant* plant);
    };
}
#endif

