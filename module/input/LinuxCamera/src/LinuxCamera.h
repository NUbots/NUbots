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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_LINUXCAMERA_H
#define MODULES_INPUT_LINUXCAMERA_H

#include <nuclear>
#include "V4L2Camera.h"

namespace module {
    namespace input {

        /**
         * @breif This module is responsible for reading data from the Darwin's Camera and emitting the resulting images.
         *
         * @details
         *    This Reactor uses it's DarwinCamera class in order to read a stream of images from the Darwin's camera and
         *    emit them out to the rest of the system. It does this using the Video4Linux2 drivers that are built into
         *    the kernel.
         *
         * @author Michael Burton
         * @author Jake Woods
         * @author Trent Houliston
         */
        class LinuxCamera : public NUClear::Reactor {

        private:
            /// @brief Our internal camera class that interacts with the physical device
            V4L2Camera camera;
        public:
            /// @brief Called by the PowerPlant to build and setup our Reactor
            LinuxCamera(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_LINUXCAMERA_H
