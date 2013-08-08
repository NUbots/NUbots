/*
 * This file is part of NUBugger.
 *
 * NUBugger is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * NUBugger is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with NUBugger.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "NUBugger.h"

#include "messages/NUAPI.pb.h"
#include "messages/DarwinSensors.h"

namespace modules {

    NUBugger::NUBugger(NUClear::PowerPlant& plant) : Reactor(plant), pub(plant.getZMQContext(), ZMQ_PUB) {

        // Set our high water mark
        int64_t hwm = 3;
        pub.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));

        // Bind to port 12000
        pub.bind("tcp://*:12000");

        // This trigger gets the output from the sensors (unfiltered)
        on<Trigger<Messages::DarwinSensors>>([this](const Messages::DarwinSensors& sensors) {

            API::Message message;

            message.set_type(API::Message::SENSOR_DATA);
            message.set_utc_timestamp(std::time(0));

            auto* sensorData = message.mutable_sensor_data();

            for (int i = 0; i < 20; ++i) {

                auto* servo = sensorData->add_motor();

                servo->set_position(sensors.servo[i].presentPosition);
                servo->set_velocity(sensors.servo[i].presentSpeed);
                servo->set_target(sensors.servo[i].goalPosition);
                servo->set_stiffness(sensors.servo[i].pGain);
                servo->set_current(sensors.servo[i].load);
                servo->set_torque(sensors.servo[i].torqueLimit);
                servo->set_temperature(sensors.servo[i].temperature);
            }

            auto serialized = message.SerializeAsString();
            pub.send(serialized.data(), serialized.size());
        });
    }
}
