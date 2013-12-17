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

#include <zmq.hpp>
#include <jpeglib.h>
#include <cxxabi.h>

#include "messages/NUAPI.pb.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include "utility/NUbugger/NUgraph.h"

#include "utility/image/ColorModelConversions.h"

using messages::platform::darwin::DarwinSensors;
using messages::input::Image;
using messages::vision::ClassifiedImage;
using NUClear::DEBUG;
using utility::NUbugger::graph;
using std::chrono::duration_cast;
using std::chrono::microseconds;

namespace modules {
	namespace support {

		NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment)
			: Reactor(std::move(environment))
			, pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB) {
			// Set our high water mark
			//int hwm = 50;
			//pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

			// Bind to port 12000
			pub.bind("tcp://*:12000");

			on<Trigger<DataPoint>>([this](const DataPoint& data_point) {
				API::Message message;
				message.set_type(API::Message::DATA_POINT);
				message.set_utc_timestamp(std::time(0));

				auto* dataPoint = message.mutable_datapoint();
				dataPoint->set_label(data_point.label);
				for (auto value : data_point.values) {
					dataPoint->add_value(value);
				}

				send(message);
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

					/*if (sensors.servo[i].errorFlags > 0) {
						std::cout << sensors.servo[i].errorFlags << std::endl;
					}*/
				}

				auto* gyro = sensorData->mutable_gyro();
				gyro->add_float_value(sensors.gyroscope.x);
				gyro->add_float_value(sensors.gyroscope.y);
				gyro->add_float_value(sensors.gyroscope.z);

				auto* accel = sensorData->mutable_accelerometer();
				accel->add_float_value(sensors.accelerometer.x);
				accel->add_float_value(sensors.accelerometer.y);
				accel->add_float_value(sensors.accelerometer.z);

				/*if (sensors.cm730ErrorFlags > 0) {
					std::cout << sensors.cm730ErrorFlags << std::endl;
				}

				if (sensors.fsr.left.errorFlags > 0) {
					std::cout << sensors.fsr.left.errorFlags << std::endl;
				}

				if (sensors.fsr.right.errorFlags > 0) {
					std::cout << sensors.fsr.right.errorFlags << std::endl;
				}*/

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

				send(message);
			});

			on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

				//std::cout << "Image!" << std::endl;

				API::Message message;
				message.set_type(API::Message::VISION);
				message.set_utc_timestamp(std::time(0));

				auto* visionData = message.mutable_vision();
				auto* imageData = visionData->mutable_image();
				std::string* imageBytes = imageData->mutable_data();
                                
				if(image.source().empty()) {
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
						uint8_t* start = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&image.raw()[i * image.width()]));
						jpeg_write_scanlines(&jpegC, &start, 1);
					}

					// Finish our compression
					jpeg_finish_compress(&jpegC);

					// Close the memory file)
					fclose(memFile);

					// Destroy the compression object (free memory)
					jpeg_destroy_compress(&jpegC);
				} else {
					// Reserve enough space in the image data to store the output
					imageBytes->resize(image.source().size());
					imageData->set_width(image.width());
					imageData->set_height(image.height());
					
					imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));
				}

				send(message);
			});

			on<Trigger<NUClear::ReactionStatistics>>([this](const NUClear::ReactionStatistics& stats) {
				API::Message message;
				message.set_type(API::Message::REACTION_STATISTICS);
				message.set_utc_timestamp(std::time(0));

				auto* reactionStatistics = message.mutable_reactionstatistics();

				//reactionStatistics->set_name(stats.name);
				reactionStatistics->set_reactionid(stats.reactionId);
				reactionStatistics->set_taskid(stats.taskId);
				reactionStatistics->set_causereactionid(stats.causeReactionId);
				reactionStatistics->set_causetaskid(stats.causeTaskId);
				reactionStatistics->set_emitted(duration_cast<microseconds>(stats.emitted.time_since_epoch()).count());
				reactionStatistics->set_started(duration_cast<microseconds>(stats.started.time_since_epoch()).count());
				reactionStatistics->set_finished(duration_cast<microseconds>(stats.finished.time_since_epoch()).count());

				/*std::string name = stats.name;
                int status = -4;
                char* res = abi::__cxa_demangle(name.c_str(), NULL, NULL, &status);
                const char* const demangled_name = (status == 0) ? res : name.c_str();
                std::string ret_val(demangled_name);
                free(res);*/

				/*log<NUClear::DEBUG>("testing! ", demangled_name);*/
				
				int status = -4; // some arbitrary value to eliminate the compiler warning
				std::unique_ptr<char, void(*)(void*)> res {
					abi::__cxa_demangle(stats.name.c_str(), nullptr, nullptr, &status),
					std::free
				};

                std::string demangled_name(status == 0 ? res.get() : stats.name );

				reactionStatistics->set_name(demangled_name);

				send(message);
			});

			on<Trigger<ClassifiedImage>>([this](const ClassifiedImage& image) {

				API::Message message;
				message.set_type(API::Message::VISION);
				message.set_utc_timestamp(std::time(0));
				API::Vision* api_vision = message.mutable_vision();

				API::VisionClassifiedImage* api_classified_image = api_vision->mutable_classified_image();

				for (auto& rowColourSegments : image.horizontal_filtered_segments.m_segmentedScans) {
					for (auto& colorSegment : rowColourSegments) {
						auto& start = colorSegment.m_start;
						auto& end = colorSegment.m_end;
						auto& colour = colorSegment.m_colour;

						API::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
						api_segment->set_start_x(start[0]);
						api_segment->set_start_y(start[1]);
						api_segment->set_end_x(end[0]);
						api_segment->set_end_y(end[1]);
						api_segment->set_colour(colour);
					}
				}

				for (auto& columnColourSegments : image.vertical_filtered_segments.m_segmentedScans)
				{
					for (auto& colorSegment : columnColourSegments)
					{
						auto& start = colorSegment.m_start;
						auto& end = colorSegment.m_end;
						auto& colour = colorSegment.m_colour;

						API::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
						api_segment->set_start_x(start[0]);
						api_segment->set_start_y(start[1]);
						api_segment->set_end_x(end[0]);
						api_segment->set_end_y(end[1]);
						api_segment->set_colour(colour);
					}
				}

				send(message);
				
			});

			// When we shutdown, close our publisher
			on<Trigger<Shutdown>>([this](const Shutdown&) {
				pub.close();
			});
		}

		/**
		 * This method needs to be used over pub.send as all calls to
		 * pub.send need to be synchronized with a concurrency primative
		 * (such as a mutex)
		 */
		void NUbugger::send(zmq::message_t& packet) {
			std::lock_guard<std::mutex> lock(mutex);
			pub.send(packet);
		}

		void NUbugger::send(API::Message message) {
			auto serialized = message.SerializeAsString();
			zmq::message_t packet(serialized.size());
			memcpy(packet.data(), serialized.data(), serialized.size());
			send(packet);
		}

	} // support
} // modules
