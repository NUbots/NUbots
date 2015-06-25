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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "messages/support/nubugger/proto/Message.pb.h"

#include "utility/time/time.h"

/**
 * @author Monica Olejniczak
 */
namespace modules {
namespace support {

    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::input::proto::Sensors;
    using messages::behaviour::proto::Behaviour;

    /**
     * @brief Provides triggers to send overview information over the network using the overview 
     * instance variable.
     */
    void NUbugger::provideOverview() {

        handles["overview"].push_back(on<Trigger<Every<5, std::chrono::seconds>>>([this](const time_t&) {
            Message message;
            message.set_type(Message::OVERVIEW);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            *message.mutable_overview() = overview;

            send(message);
        }));

        handles["overview"].push_back(on<Trigger<Behaviour::State>>([this](const Behaviour::State& state) {
            overview.set_behaviour_state(state);
        }));

        handles["overview"].push_back(on<Trigger<Sensors>>([this](const Sensors& sensors) {
            overview.set_voltage(sensors.voltage());
            overview.set_battery(sensors.battery());
        }));

    }
}
}