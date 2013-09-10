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

#ifndef MODULES_SIGNALCATCHER_H
#define MODULES_SIGNALCATCHER_H

#include <NUClear.h>
#include <exception>

namespace modules {

    struct SegmentationFault : public std::exception{};

    /**
     * @brief Handles OS interrupt signals.
     *
     * @details
     *  This module catches SIGINT and SIGSEGV and changed how
     *  they are handled. Only one SignalCatcher should ever be
     *  installed in the program and this module will be incompatible
     *  with any code that uses signal(SIGINT, ...) or signal(SIGSEGV, ...)
     *  anywhere else. Additionally if there are multiple NUClear::PowerPlant
     *  instances this module may cause a race condition and incorrectly shut down
     *  the wrong power plant.
     */
    class SignalCatcher : public NUClear::Reactor {
    public:
        explicit SignalCatcher(NUClear::PowerPlant* plant);
    private:
        static NUClear::PowerPlant* powerPlant;

        static void sigintHandler(int signal);
        static void segfaultConverter(int signal);
    };
}
#endif

