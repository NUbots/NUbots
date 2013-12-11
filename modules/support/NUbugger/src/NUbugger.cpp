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
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Image.h"
#include "messages/localisation/FieldObject.h"
#include "utility/NUbugger/NUgraph.h"

#include "utility/image/ColorModelConversions.h"

using messages::platform::darwin::DarwinSensors;
using messages::input::Image;
using NUClear::DEBUG;
using utility::NUbugger::graph;

namespace modules {
	namespace support {

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

				void send(API::Message message) {
					auto serialized = message.SerializeAsString();
					zmq::message_t packet(serialized.size());
					memcpy(packet.data(), serialized.data(), serialized.size());
					send(packet);
				}
		};

		NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
			// Set our high water mark
			int hwm = 3;
			m->pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

			// Bind to port 12000
			m->pub.bind("tcp://*:12000");

			on<Trigger<DataPoint>>([this](const DataPoint& data_point) {
				API::Message message;
				message.set_type(API::Message::DATA_POINT);
				message.set_utc_timestamp(std::time(0));

				auto* dataPoint = message.mutable_datapoint();
				dataPoint->set_label(data_point.label);
				for (auto value : data_point.values) {
					dataPoint->add_value(value);
				}

				m->send(message);
			});

			// This trigger gets the output from the sensors (unfiltered)
			on<Trigger<DarwinSensors>>([this](const DarwinSensors& sensors) {
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
				accel->add_float_value(sensors.accelerometer.x);
				accel->add_float_value(sensors.accelerometer.y);
				accel->add_float_value(sensors.accelerometer.z);

				emit(graph(
					"Accelerometer", 
					sensors.accelerometer.x,
					sensors.accelerometer.y,
					sensors.accelerometer.z
					
				));

				emit(graph(
					"Gyro",
					sensors.gyroscope.x,
					sensors.gyroscope.y,
					sensors.gyroscope.z
				));

				auto* orient = sensorData->mutable_orientation();
				orient->add_float_value(0);
				orient->add_float_value(0);
				orient->add_float_value(0);

				m->send(message);
			});

			on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

				API::Message message;
				message.set_type(API::Message::VISION);
				message.set_utc_timestamp(std::time(0));
				auto* visionData = message.mutable_vision();
				auto* imageData = visionData->mutable_image();
				std::string* imageBytes = imageData->mutable_data();

				// Reserve enough space in the image data to store the output
				imageBytes->resize(image.width() * image.height());
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
                                
                                // Compress each row
                                for(size_t i = 0; i < image.height(); ++i) {
                                    uint8_t* start = reinterpret_cast<uint8_t*>(image.raw().get() + i * image.width());
                                    jpeg_write_scanlines(&jpegC, &start, 1);
                                }
                                
				// Finish our compression
				jpeg_finish_compress(&jpegC);

				// Close the memory file)
				fclose(memFile);

				// Destroy the compression object (free memory)
				jpeg_destroy_compress(&jpegC);

				m->send(message);
			});

			on<Trigger<messages::localisation::FieldObject>>([this](const messages::localisation::FieldObject& field_object) {
				API::Message message;

				message.set_type(API::Message::LOCALISATION);
				message.set_utc_timestamp(std::time(0));

				auto* localisation = message.mutable_localisation();
				auto* api_ball = localisation->add_field_object();

				api_ball->set_name(field_object.name);
				api_ball->set_wm_x(field_object.wm_x);
				api_ball->set_wm_y(field_object.wm_y);
				api_ball->set_sd_x(field_object.sd_x);
				api_ball->set_sd_y(field_object.sd_y);
				api_ball->set_sr_xx(field_object.sr_xx);
				api_ball->set_sr_xy(field_object.sr_xy);
				api_ball->set_sr_yy(field_object.sr_yy);
				api_ball->set_lost(field_object.lost);

				m->send(message);
			});

			// When we shutdown, close our publisher
			on<Trigger<Shutdown>>([this](const Shutdown&) {
				m->pub.close();
			});
		}

	} // support
} // modules
