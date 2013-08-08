/*
 * This file is part of DarwinPlatform.
 *
 * DarwinPlatform is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DarwinPlatform is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DarwinPlatform.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_DARWINPLATFORM_H
#define MODULES_DARWINPLATFORM_H

#include <NUClear.h>
#include <zmq.hpp>

namespace modules {

    class NUBugger : public NUClear::Reactor {
    private:
        /// @brief our zmq pub socket
        zmq::socket_t pub;
        
    public:
        explicit NUBugger(NUClear::PowerPlant& plant);
    };
}
#endif

