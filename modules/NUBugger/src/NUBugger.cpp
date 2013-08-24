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
#include "jpge.h"

#include "messages/NUAPI.pb.h"
#include "messages/DarwinSensors.h"
#include "messages/Image.h"

#include "utility/image/ColorModelConversions.h"

using utility::image::RGB;
using utility::image::YCbCr;

namespace modules {

    NUBugger::NUBugger(NUClear::PowerPlant* plant) : Reactor(plant), pub(NUClear::Extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB) {

        // Set our high water mark
        int64_t hwm = 3;
        pub.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));

        // Bind to port 12000
        pub.bind("tcp://*:12000");

        // This trigger gets the output from the sensors (unfiltered)
        on<Trigger<messages::DarwinSensors>>([this](const messages::DarwinSensors& sensors) {

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

            auto* gyro = sensorData->mutable_gyro();
            gyro->add_float_value(sensors.gyroscope.x);
            gyro->add_float_value(sensors.gyroscope.y);
            gyro->add_float_value(sensors.gyroscope.z);
            
            auto* accel = sensorData->mutable_accelerometer();
            accel->add_float_value(sensors.acceleronometer.x);
            accel->add_float_value(sensors.acceleronometer.y);
            accel->add_float_value(sensors.acceleronometer.z);
            
            auto* orient = sensorData->mutable_orientation();
            orient->add_float_value(0);
            orient->add_float_value(0);
            orient->add_float_value(0);
            
            auto serialized = message.SerializeAsString();
            zmq::message_t packet(serialized.size());
            memcpy(packet.data(), serialized.data(), serialized.size());
            pub.send(packet);
        });

        on<Trigger<messages::Image>>([this](const messages::Image& image) {
            API::Message message;
            message.set_type(API::Message::VISION);
            message.set_utc_timestamp(std::time(0));
            auto* visionData = message.mutable_vision();
            auto* imageData = visionData->mutable_image();

            int imageWidth = imageData->width();
            int imageHeight = imageData->height();
            jpge::uint8* data = new jpge::uint8[imageWidth * imageHeight * 3]();
            
            int index = 0;
            for (int y = imageHeight - 1; y >= 0; y--) {
                for (int x = imageWidth - 1; x >= 0; x--) {
                messages::Image::Pixel pixel = image(x, y);
                RGB rgb = toRGB(YCbCr{ pixel.y, pixel.cb, pixel.cr });
                    /*unsigned char r, g, b;
                    ColorModelConversions::fromYCbCrToRGB(
                        (unsigned char) pixel.y, 
                        (unsigned char) pixel.cb, 
                        (unsigned char) pixel.cr, 
                        r, 
                        g, 
                        b
                    );*/

                    data[index] = rgb.r;
                    data[index + 1] = rgb.g;
                    data[index + 2] = rgb.b;

                    index += 3;
                }
            }

            // allocate a buffer that's hopefully big enough (this is way overkill for jpeg)
            int orig_buf_size = imageWidth * imageHeight * 3; 
            if (orig_buf_size < 1024) orig_buf_size = 1024;
            void *pBuf = malloc(orig_buf_size);
            int c_size = orig_buf_size;
            jpge::compress_image_to_jpeg_file_in_memory(pBuf, c_size, imageWidth, imageHeight, 3, data);

            delete[] data;

            imageData->set_width(imageWidth);
            imageData->set_height(imageHeight);
            imageData->set_data(pBuf, c_size);

            free(pBuf);

            auto serialized = message.SerializeAsString();
            zmq::message_t packet(serialized.size());
            memcpy(packet.data(), serialized.data(), serialized.size());
            pub.send(packet);
        });


        // When we shutdown, close our publisher
        on<Trigger<Shutdown>>([this](const Shutdown&) {
            pub.close();
        });
    }
}
