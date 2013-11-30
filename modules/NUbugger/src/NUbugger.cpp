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

#include "NUbugger.h"
#include "utility/idiom/pimpl_impl.h"

#include <zmq.hpp>
#include <jpeglib.h>

#include "messages/NUAPI.pb.h"
#include "messages/DarwinSensors.h"
#include "messages/Image.h"

#include "utility/image/ColorModelConversions.h"

using utility::image::RGB;
using utility::image::YCbCr;

namespace modules {
    // Implement our impl class as per the pimpl idiom.
    class NUbugger::impl {
        public:
            zmq::socket_t pub;

            std::mutex mutex;

            impl() : pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB) {}

            /**
             * This method needs to be used over pub.send as all calls to
             * pub.send need to be synchronized with a concurrency primative
             * (such as a mutex)
             */
            void send(zmq::message_t& packet) {
                std::lock_guard<std::mutex> lock(mutex);
                pub.send(packet);
            }
    };

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        // Set our high water mark
        int64_t hwm = 3;
        m->pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

        // Bind to port 12000
        m->pub.bind("tcp://*:12000");

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
            m->send(packet);
        });

        on<Trigger<messages::Image>, Options<Single, Priority<NUClear::LOW>>>([this](const messages::Image& image) {

            API::Message message;
            message.set_type(API::Message::VISION);
            message.set_utc_timestamp(std::time(0));
            auto* visionData = message.mutable_vision();
            auto* imageData = visionData->mutable_image();
            std::string* imageBytes = imageData->mutable_data();

            // Reserve enough space in the image data to store the output
            imageBytes->resize(image.size());
            imageData->set_width(image.width());
            imageData->set_height(image.height());

            // Open the memory of the string as a file (using some dastardly hackery)
            auto memFile = fmemopen(const_cast<char*>(imageBytes->data()), imageBytes->size(), "wb");

            // Our jpeg compression structures
            jpeg_compress_struct jpegC;
            jpeg_error_mgr jpegErr;

            // Set our error handler on our Config object
            jpegC.err = jpeg_std_error(&jpegErr);

            // Create our compression object
            jpeg_create_compress(&jpegC);

            // Set our output to the file
            jpeg_stdio_dest(&jpegC, memFile);

            // Set information about our image
            jpegC.image_width = image.width();
            jpegC.image_height = image.height();
            jpegC.input_components = 3;
            jpegC.in_color_space = JCS_YCbCr;

            // Set the default compression parameters
            jpeg_set_defaults(&jpegC);

            // Start compression
            jpeg_start_compress(&jpegC, true);

            // Allocate some space for our row data
            std::unique_ptr<JSAMPLE[]> row = std::unique_ptr<JSAMPLE[]>(new JSAMPLE[image.width() * 3]);

            // Read in each scanline
            while (jpegC.next_scanline < jpegC.image_height) {
                // Allocate a pointer to our row (since it likes to have them sparse)
                JSAMPLE* rowPtr = row.get();
                JSAMPARRAY ptr = &rowPtr;

                // Now load the data into the row from image
                for(size_t i = 0; i < image.width(); ++i) {
                    row[i * 3 + 0] = image(i, jpegC.next_scanline).y;
                    row[i * 3 + 1] = image(i, jpegC.next_scanline).cb;
                    row[i * 3 + 2] = image(i, jpegC.next_scanline).cr;
                }

                // Write this scanline in
                jpeg_write_scanlines(&jpegC, ptr, 1);
            }

            // Finish our compression
            jpeg_finish_compress(&jpegC);

            // Close the memory file)
            fclose(memFile);

            // Destroy the compression object (free memory)
            jpeg_destroy_compress(&jpegC);

            auto serialized = message.SerializeAsString();
            zmq::message_t packet(serialized.size());
            memcpy(packet.data(), serialized.data(), serialized.size());
            m->send(packet);
        });

        // When we shutdown, close our publisher
        on<Trigger<Shutdown>>([this](const Shutdown&) {
            m->pub.close();
        });
    }

}
