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

#ifndef MODULES_SUPPORT_SIGNALCATCHER_H
#define MODULES_SUPPORT_SIGNALCATCHER_H

#include <nuclear>
#include <exception>

#include "message/support/SegmentationFault.h"

namespace module {
    namespace support {

        /**
         * @brief Handles OS interrupt signals.
         *
         * @details
         *  This module catches SIGINT and SIGSEGV and changes how they are handled. Only one SignalCatcher should ever be
         *  installed in the program and this module will be incompatible with any code that uses signal(SIGINT, ...)
         *  or signal(SIGSEGV, ...) anywhere else. Additionally if there are multiple NUClear::PowerPlant instances this
         *  module may cause a race condition and incorrectly shut down the wrong power plant.
         *
         * @author Trent Houliston
         */
        class SignalCatcher : public NUClear::Reactor {
        public:
            explicit SignalCatcher(std::unique_ptr<NUClear::Environment> environment);
        private:

            /// Our static PowerPlant variable that we use to shutdown the system
            static NUClear::PowerPlant* POWER_PLANT;

            /// This boolean is set to true if the user sends sigint, the second time this is sent exit(1) is called
            static volatile bool userRequestedShutdown;

            /**
             * @brief TODO
             *
             * @details
             *  TODO
             *
             * @param signal the signal that was passed to this signal handler
             */
            static void sigintHandler(int signal);

            /**
             * @brief TODO
             *
             * @details
             *  TODO
             *
             * @param signal the signal that was passed to this signal handler
             */
            static void segfaultConverter(int signal);
        };

    }  // support
}  // modules

#endif  // MODULES_SUPPORT_SIGNALCATCHER_H

