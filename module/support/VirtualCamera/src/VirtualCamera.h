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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_SUPPORT_VIRTUALCAMERA_H
#define MODULES_SUPPORT_VIRTUALCAMERA_H

#include <Eigen/Core>
#include <nuclear>
#include <string>

#include "message/input/Image.h"

namespace module {
namespace support {

    class VirtualCamera : public NUClear::Reactor {
        ReactionHandle emitImageHandle;
        std::string imagePath;

    public:
        /// @brief Called by the powerplant to build and setup the VirtualCamera reactor.
        explicit VirtualCamera(std::unique_ptr<NUClear::Environment> environment);

    private:
        Eigen::Matrix4d Hcw;
        message::input::Image::Lens lens;
    };
}  // namespace support
}  // namespace module

#endif
