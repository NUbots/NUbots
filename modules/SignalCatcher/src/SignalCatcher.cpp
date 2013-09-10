/*
 * This file is part of SignalCatcher.
 *
 * SignalCatcher is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * SignalCatcher is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with SignalCatcher.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "SignalCatcher.h"
#include <signal.h>

namespace modules {

    SignalCatcher::SignalCatcher(NUClear::PowerPlant* plant) : Reactor(plant) {
        powerPlant = plant;

        signal(SIGINT, &SignalCatcher::sigintHandler);
        signal(SIGSEGV, &SignalCatcher::segfaultConverter);
    }

    void SignalCatcher::sigintHandler(int signal) {
        std::cout << std::endl << "Shutdown Command Sent" << std::endl;

        if(!run) {
            powerPlant->shutdown();
            run = true;
        } 
        else {
            exit(1);
        }
    }

    void SignalCatcher::segfaultConverter(int signal) {
        std::cout << "Segmentation Fault" << std::endl;
        throw SegmentationFault();
    }

}
