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

#ifndef MODUlES_INPUT_V4L2CAMERASETTING_H
#define MODUlES_INPUT_V4L2CAMERASETTING_H

#include <cstdint>

namespace module {
    namespace input {

        /**
         * @brief Holds an individual camera setting encapsulating the ioctl calls needed to get and set it
         *
         * @author Trent Houliston
         */
        class V4L2CameraSetting {
        private:
            /// @brief the file descriptor for our camera
            int fd;
            /// @brief the ID for the setting we are manipulating
            unsigned int id;
        public:
            /**
             * Builds a new camera setting, encapsulating the setting of setting "id" for file descriptor "fileDescriptor"
             *
             * @param fileDescriptor    the file descriptor to the camera we are setting for
             * @param id                the id for the setting we are setting
             */
            V4L2CameraSetting(int fileDescriptor, unsigned int id);

            /**
             * Gets the current value of this setting
             *
             * @return the current value for this setting
             */
            int32_t get();

            /**
             * Sets the current value for this setting
             *
             * @param the value to set this setting to
             *
             * @return true if we succeeded in setting this value, false otherwise
             */
            bool set(int32_t);
        };
    }  // input
}  // modules

#endif  // MODUlES_INPUT_V4L2CAMERASETTING_H

