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

#include "NUbugger.h"

#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/input/Sensors.h"

#include "utility/support/proto_armadillo.h"
#include "utility/time/time.h"

namespace modules {
namespace support {
    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::input::Sensors;

    void NUbugger::provideSensors() {

        // This trigger gets the output from the sensors (unfiltered)
        handles[Message::SENSOR_DATA].push_back(on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

            Message message;

            message.set_type(Message::SENSOR_DATA);
            message.set_filter_id(1);
            message.set_utc_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(sensors.timestamp.time_since_epoch()).count());

            auto* sensorData = message.mutable_sensor_data();

            sensorData->set_timestamp(sensors.timestamp.time_since_epoch().count());
            sensorData->set_voltage(sensors.voltage);
            sensorData->set_battery(sensors.battery);

            // Add each of the servos into the protocol buffer
            for(const auto& s : sensors.servos) {

                auto* servo = sensorData->add_servo();

                servo->set_error_flags(s.errorFlags);

                servo->set_id(static_cast<messages::input::proto::Sensors_ServoID>(s.id));

                servo->set_enabled(s.enabled);

                servo->set_p_gain(s.pGain);
                servo->set_i_gain(s.iGain);
                servo->set_d_gain(s.dGain);

                servo->set_goal_position(s.goalPosition);
                servo->set_goal_velocity(s.goalVelocity);

                servo->set_present_position(s.presentPosition);
                servo->set_present_velocity(s.presentVelocity);

                servo->set_load(s.load);
                servo->set_voltage(s.voltage);
                servo->set_temperature(s.temperature);
            }

            // The gyroscope values (x,y,z)
            *sensorData->mutable_gyroscope() << sensors.gyroscope;

            // The accelerometer values (x,y,z)
            *sensorData->mutable_accelerometer() << sensors.accelerometer;

            // The orientation matrix
            *sensorData->mutable_orientation() << sensors.orientation;

            // TODO: these do not exist in Sensors.h, this needs reimplementing
            // The left FSR values
            // auto* lfsr = sensorData->mutable_left_fsr();
            // lfsr->set_x(sensors.leftFSR[0]);
            // lfsr->set_y(sensors.leftFSR[1]);
            // lfsr->set_z(sensors.leftFSR[2]);

            // // The right FSR values
            // auto* rfsr = sensorData->mutable_right_fsr();
            // rfsr->set_x(sensors.rightFSR[0]);
            // rfsr->set_y(sensors.rightFSR[1]);
            // rfsr->set_z(sensors.rightFSR[2]);

            // The LEDs
            for(auto& l : sensors.leds) {
                auto* led = sensorData->add_led();
                led->set_id(l.id);
                led->set_colour(l.colour);
            }

            // The Buttons
            for(auto& b : sensors.buttons) {
                auto* button = sensorData->add_button();
                button->set_id(b.id);
                button->set_value(b.value);
            }

            send(message);

        }));
    }
}
}
