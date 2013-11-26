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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_PLATFORM_DARWIN_HARDWAREIO_H
#define MODULES_PLATFORM_DARWIN_HARDWAREIO_H

#include <NUClear.h>

#include "darwin/Darwin.h"

namespace modules {
namespace Platform {
namespace Darwin {

    /**
     * This NUClear Reactor is responsible for reading in the data for the Darwin Platform and emitting it to the rest
     * of the system
     *
     * @author Trent Houliston
     */
    class HardwareIO : public NUClear::Reactor {
    private:
        /// @brief Our internal darwin class that is used for interacting with the hardware
        darwin::Darwin darwin;

    public:
        /// @brief called by a Powerplant to construct this reactor
        explicit HardwareIO(std::unique_ptr<NUClear::Environment> environment);
    };
}
}
}
#endif

