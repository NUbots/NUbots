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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "SignalCatcher.h"

#include <csignal>

namespace module {
namespace support {

    // Set our initial shutdown request state
    volatile bool userRequestedShutdown = false;

    // Initialize our powerplant variable
    NUClear::PowerPlant* POWER_PLANT = nullptr;

    void sigint(int) {

        // Output that a shutdown command was sent (so the user knows the ctrl-c worked)
        std::cout << std::endl << "Shutdown Command Sent" << std::endl;

        // If this is the first time they asked
        if (!userRequestedShutdown) {

            // Ask the system to shutdown, and flag that the user has asked once
            POWER_PLANT->shutdown();
            userRequestedShutdown = true;
        }
        // If this is the second time, kill everything
        else {
            exit(1);
        }
    }

    // Our segmentation fault converter function
    void sigsegv(int) {

        throw std::runtime_error("Segmentation Fault");
    }

    void sigabrt(int) {

        throw std::runtime_error("Abort signal");
    }

    SignalCatcher::SignalCatcher(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Store our powerplant in the static variable
        POWER_PLANT = &powerplant;
        struct sigaction action;

        // Setup our segmentation fault signal handler/converter
        std::memset(&action, 0, sizeof(action));
        action.sa_handler = sigsegv;
        action.sa_flags   = SA_NODEFER;
        sigaction(SIGSEGV, &action, nullptr);

        // Setup our abort signal handler/converter
        std::memset(&action, 0, sizeof(action));
        action.sa_handler = sigabrt;
        action.sa_flags   = SA_NODEFER;
        sigaction(SIGABRT, &action, nullptr);

        // On sigint run the sigint handler
        std::memset(&action, 0, sizeof(action));
        action.sa_handler = sigint;
        action.sa_flags   = SA_NODEFER;
        sigaction(SIGINT, &action, nullptr);
    }

}  // namespace support
}  // namespace module
