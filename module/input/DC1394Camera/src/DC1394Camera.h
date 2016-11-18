/*
 * This file is part of NUbots Codebase.
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

#ifndef MODULE_INPUT_DC1394CAMERA_H
#define MODULE_INPUT_DC1394CAMERA_H

#include <nuclear>

#include <dc1394/dc1394.h>

namespace module {
namespace input {

    class DC1394Camera : public NUClear::Reactor {
    public:
        std::unique_ptr<dc1394_t, std::function<void (dc1394_t*)>> context;
        std::map<uint64_t, std::unique_ptr<dc1394camera_t, std::function<void (dc1394camera_t*)>>> cameras;

        static constexpr const char* CONFIGURATION_PATH = "Cameras";

        /// @brief Called by the powerplant to build and setup the DC1394Camera reactor.
        explicit DC1394Camera(std::unique_ptr<NUClear::Environment> environment);
    };

}
}


#endif