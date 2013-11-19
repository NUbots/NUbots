/*
 * This file is part of NUbugger.
 *
 * NUbugger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUbugger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUbugger.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_NUBUGGER_H
#define MODULES_NUBUGGER_H

#include <NUClear.h>
#include "utility/idiom/pimpl.h"

namespace modules {

    /**
     * TODO document
     *
     * @author Brendan Annable
     * @author Trent Houliston
     */
    class NUbugger : public NUClear::Reactor {
    private:
        class impl;
        utility::idiom::pimpl<impl> m;

        /// @brief our zmq pub socket
        /*zmq::socket_t pub;

        std::mutex mutex;*/

        /**
         * This method needs to be used over pub.send as all calls to
         * pub.send need to be synchronized with a concurrency primative
         * (such as a mutex)
         */
        //void send(zmq::message_t& packet);

    public:
        explicit NUbugger(NUClear::PowerPlant* plant);
    };
}
#endif

