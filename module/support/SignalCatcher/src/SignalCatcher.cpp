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

#include "SignalCatcher.h"
#include <csignal>

namespace module {
    namespace support {

        // Set our initial shutdown request state
        volatile bool SignalCatcher::userRequestedShutdown = false;

        // Initialize our powerplant variable
        NUClear::PowerPlant* SignalCatcher::POWER_PLANT = nullptr;

        SignalCatcher::SignalCatcher(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // Store our powerplant in the static variable
            POWER_PLANT = &powerplant;

            // On sigint run the sigint handler
            std::signal(SIGINT, &SignalCatcher::sigintHandler);

            // On a segfault run the sigsev handler
            std::signal(SIGSEGV, &SignalCatcher::segfaultConverter);

            // On a segfault run the sigsev handler
            std::signal(SIGABRT, &SignalCatcher::segfaultConverter);
        }

        void SignalCatcher::sigintHandler(int) {

            // Output that a shutdown command was sent (so the user knows the ctrl-c worked)
            std::cout << std::endl << "Shutdown Command Sent" << std::endl;

            // If this is the first time they asked
            if(!userRequestedShutdown) {

                // Ask the system to shutdown, and flag that the user has asked once
                POWER_PLANT->shutdown();
                userRequestedShutdown = true;
            }
            // If this is the second time, kill everything
            else {
                exit(1);
            }
        }

        void SignalCatcher::segfaultConverter(int) {
            throw message::support::SegmentationFault();
        }

    }  // support
}  // modules