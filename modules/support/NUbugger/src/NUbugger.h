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

#ifndef MODULES_SUPPORT_NUBUGGER_H
#define MODULES_SUPPORT_NUBUGGER_H

#include <nuclear>
#include <zmq.hpp>
#include "messages/support/NUbugger/proto/Message.pb.h"

namespace modules {
    namespace support {

        /**
         * Intelligent debugging, logging and graphing for the NUBots system.
         *
         * @author Brendan Annable
         * @author Trent Houliston
         */
        class NUbugger : public NUClear::Reactor {
        private:
            zmq::socket_t pub;

            std::mutex mutex;

            void send(zmq::message_t& packet);
            void send(messages::support::NUbugger::proto::Message message);
        public:
            explicit NUbugger(std::unique_ptr<NUClear::Environment> environment);
        };
        
    }  // support
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

