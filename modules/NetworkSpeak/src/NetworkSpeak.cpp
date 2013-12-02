/*
 * This file is part of NetworkSpeak.
 *
 * NetworkSpeak is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NetworkSpeak is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NetworkSpeak.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NetworkSpeak.h"
#include "messages/Say.h"
#include "messages/NetSpeak.pb.h"

namespace modules {

    NetworkSpeak::NetworkSpeak(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Network<messages::NetSpeak>>>([this](const Network<messages::NetSpeak>& message) {
            
            log<NUClear::DEBUG>("Saying: \"", message.data->message(), "\"");
            
            //send the message from the network to the espeak module
            emit(std::make_unique<messages::Say>(message.data->message()));
        });
    }
}
