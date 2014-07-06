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

#ifndef MODULES_SUPPORT_NUBUGGER_H
#define MODULES_SUPPORT_NUBUGGER_H

#include <nuclear>
#include <zmq.hpp>
#include "messages/localisation/FieldObject.h"
#include "messages/support/nubugger/proto/Message.pb.h"

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
            zmq::socket_t sub;

            bool listening = true;

            // Reaction Handles
            ReactionHandle dataPointHandle;
            ReactionHandle actionStartHandle;
            ReactionHandle actionKillHandle;
            ReactionHandle registerActionHandle;
            ReactionHandle sensorsHandle;
            ReactionHandle imageHandle;
            ReactionHandle reactionStatisticsHandle;
            ReactionHandle classifiedImageHandle;
            ReactionHandle goalsHandle;
            ReactionHandle ballsHandle;
            ReactionHandle localisationHandle;

            std::mutex mutex;

            void send(zmq::message_t& packet);
            void send(messages::support::nubugger::proto::Message message);

            void recvMessage(const messages::support::nubugger::proto::Message& message);
            void recvCommand(const messages::support::nubugger::proto::Message& message);
            void recvLookupTable(const messages::support::nubugger::proto::Message& message);
            void recvReactionHandlers(const messages::support::nubugger::proto::Message& message);

            void EmitLocalisationModels(
                const std::unique_ptr<messages::localisation::FieldObject>& robot_model,
                const std::unique_ptr<messages::localisation::FieldObject>& ball_model);

            void run();
            void kill();
        public:
            explicit NUbugger(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // support
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

