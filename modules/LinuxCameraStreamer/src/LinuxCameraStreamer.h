/*
 * This file is part of DarwinCameraReader.
 *
 * DarwinCameraReader is free software: you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * DarwinCameraReader is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinCameraReader.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Michael Burton <michael.burton@uon.edu.au>, Jake Woods <jake.f.woods@gmail.com>,
 * Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_DARWINCAMERAREADER_H
#define MODULES_DARWINCAMERAREADER_H

#include <NUClear.h>

#include "V4L2Camera.h"

namespace modules {
    /**
     * @breif This module is responsible for reading data from the darwins Camera and emitting the resulting images.
     *
     * @details
     *    This Reactor uses it's DarwinCamera class in order to read a stream of images from the darwin's camera and
     *    emit them out to the rest of the system. It does this using the Video4Linux2 drivers that are built into
     *    the kernel.
     *
     * @author Michael Burton
     * @author Jake Woods
     * @author Trent Houliston
     */
    class LinuxCameraStreamer : public NUClear::Reactor {

    private:
        /// @brief Our internal camera class that interacts with the physical device
        V4L2Camera camera;
    public:
        /// @brief Our configuration file for this class
        static constexpr const char* CONFIGURATION_PATH = "LinuxCameraStreamer.json";

        /// @brief Called by the powerplant to build and setup our CameraReader
        LinuxCameraStreamer(NUClear::PowerPlant* plant);
    };
}
#endif

