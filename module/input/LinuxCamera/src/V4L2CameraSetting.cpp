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

#include "V4L2CameraSetting.h"

#include <stdexcept>
#include <system_error>

#include <linux/videodev2.h>
#include <sys/ioctl.h>

namespace module {
    namespace input {

        V4L2CameraSetting::V4L2CameraSetting(int fileDescriptor, unsigned int id) : fd(fileDescriptor), id(id) {
        }

        int32_t V4L2CameraSetting::get() {

            // Check if we can access the value
            struct v4l2_queryctrl queryctrl;
            queryctrl.id = id;
            if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while attempting to get the status of this camera value");
            }
            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
                throw std::runtime_error("Requested camera value is not available");
            }
            if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
                throw std::runtime_error("Requested camera value is not supported");
            }

            // Try to get the value
            struct v4l2_control control_s;
            control_s.id = id;
            if (ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0) {
                throw std::system_error(errno, std::system_category(), "There was an error while trying to get the current value");
            }

            return control_s.value;
        }

        bool V4L2CameraSetting::set(int32_t value) {

            // Check if we can access the value
            struct v4l2_queryctrl queryctrl;
            queryctrl.id = id;
            if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
                return false;
            }
            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
                return false;
            }
            if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
                return false;
            }

            // Shape the value if it's above or below our limits
            if (value < queryctrl.minimum) {
                value = queryctrl.minimum;
            }
            if (value > queryctrl.maximum) {
                value = queryctrl.maximum;
            }

            // Attempt to write the value
            struct v4l2_control control_s;
            control_s.id = id;
            control_s.value = value;
            if (ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
                return false;
            }

            // We succeeded
            return true;
        }

    }  // input
}  // modules