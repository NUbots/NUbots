/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "SignalCatcher.hpp"

#include <csignal>
#include <iostream>

namespace module::support {

    // Set our initial shutdown request state
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    volatile bool userRequestedShutdown = false;

    // Initialize our powerplant variable
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    NUClear::PowerPlant* POWER_PLANT = nullptr;

    void sigint(int /*signal*/) {

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
    void sigsegv(int /*signal*/) {

        throw std::runtime_error("Segmentation Fault");
    }

    void sigabrt(int /*signal*/) {

        throw std::runtime_error("Abort signal");
    }

    SignalCatcher::SignalCatcher(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Store our powerplant in the static variable
        POWER_PLANT = &powerplant;
        struct sigaction action {};

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
}  // namespace module::support
