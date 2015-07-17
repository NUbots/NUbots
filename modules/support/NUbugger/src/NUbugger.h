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
#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/gameevents/GameEvents.h"

namespace modules {
    namespace support {

        /**
         * Intelligent debugging, logging and graphing for the NUbots system.
         *
         * @author Brendan Annable
         * @author Trent Houliston
         */
        class NUbugger : public NUClear::Reactor {
        private:
            zmq::socket_t pub;
            zmq::socket_t sub;

            uint pubPort = 0;
            uint subPort = 0;

            NUClear::clock::duration max_image_duration;
            NUClear::clock::time_point last_image = NUClear::clock::now();
            NUClear::clock::duration max_classified_image_duration;
            NUClear::clock::time_point last_classified_image = NUClear::clock::now();

            bool listening = true;

            // Reaction Handles
            std::map<messages::support::nubugger::proto::Message::Type, std::vector<ReactionHandle>> handles;

            std::map<std::string, uint> dataPointFilterIds;
            uint dataPointFilterId = 1;

            // Send control
            bool networkEnabled = false;
            bool fileEnabled = false;

            messages::support::nubugger::proto::Overview overview;
            messages::behaviour::proto::Subsumption subsumption;

            std::ofstream outputFile;

            std::mutex networkMutex;
            std::mutex fileMutex;


            void provideOverview();
            void provideDataPoints();
            void provideDrawObjects();
            void provideSubsumption();
            void provideGameController();
            void provideLocalisation();
            void provideReactionStatistics();
            void provideSensors();
            void provideVision();

            void sendReactionHandles();

            void sendGameState(std::string event, const messages::input::gameevents::GameState& gameState);
            messages::input::proto::GameState::Data::Phase getPhase(const messages::input::gameevents::Phase& phase);
            messages::input::proto::GameState::Data::Mode getMode(const messages::input::gameevents::Mode& phase);
            messages::input::proto::GameState::Data::PenaltyReason getPenaltyReason(const messages::input::gameevents::PenaltyReason& penaltyReason);

            void sendConfigurationState();
            void sendSubsumption();

            void send(zmq::message_t& packet);
            void send(messages::support::nubugger::proto::Message message);

            messages::support::nubugger::proto::Message createMessage(messages::support::nubugger::proto::Message::Type type, uint filterId=0);

            void recvMessage(const messages::support::nubugger::proto::Message& message);
            void recvCommand(const messages::support::nubugger::proto::Message& message);
            void recvLookupTable(const messages::support::nubugger::proto::Message& message);
            void recvReactionHandles(const messages::support::nubugger::proto::Message& message);
            void recvConfigurationState(const messages::support::nubugger::proto::Message& message);

            void EmitLocalisationModels(
                const std::unique_ptr<messages::localisation::FieldObject>& robot_model,
                const std::unique_ptr<messages::localisation::FieldObject>& ball_model);

            void run();
            void kill();

            messages::support::nubugger::proto::Message::Type getMessageTypeFromString(std::string type_name);
            std::string getStringFromMessageType(messages::support::nubugger::proto::Message::Type type);
        public:
            static constexpr const char* CONFIGURATION_PATH = "NUbugger.yaml";
            static constexpr const char* IGNORE_TAG = "IGNORE";
            explicit NUbugger(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // support
}  // modules

#endif  // MODULES_SUPPORT_NUBUGGER_H

