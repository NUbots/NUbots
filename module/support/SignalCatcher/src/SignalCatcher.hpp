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

#ifndef MODULES_SUPPORT_SIGNALCATCHER_HPP
#define MODULES_SUPPORT_SIGNALCATCHER_HPP

#include <exception>
#include <nuclear>

namespace module::support {

    /**
     * @brief Handles OS interrupt signals.
     *
     * @details
     *  This module catches SIGINT and SIGSEGV and changes how they are handled. Only one SignalCatcher should ever
     * be installed in the program and this module will be incompatible with any code that uses signal(SIGINT, ...)
     *  or signal(SIGSEGV, ...) anywhere else. Additionally if there are multiple NUClear::PowerPlant instances this
     *  module may cause a race condition and incorrectly shut down the wrong power plant.
     *
     * @author Trent Houliston
     */
    class SignalCatcher : public NUClear::Reactor {
    public:
        explicit SignalCatcher(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::support

#endif  // MODULES_SUPPORT_SIGNALCATCHER_HPP
