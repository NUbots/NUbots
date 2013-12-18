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
using messages::support::NUbugger::proto::Message;

namespace modules {
	namespace support {

		NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment)
			: Reactor(std::move(environment))
			, pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB) {
			// Set our high water mark
			int hwm = 50;
			pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

			// Bind to port 12000
			pub.bind("tcp://*:12000");

			on<Trigger<DataPoint>>([this](const DataPoint& data_point) {
				Message message;
				message.set_type(Message::DATA_POINT);
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
				Message message;

				message.set_type(Message::SENSOR_DATA);
				message.set_utc_timestamp(std::time(0));

				auto* sensorData = message.mutable_sensor_data();

				for (int i = 0; i < 20; ++i) {

					auto* servo = sensorData->add_servo();

					servo->set_error_flags(sensors.servo[i].errorFlags);

					servo->set_id(static_cast<messages::input::proto::Sensors_ServoID>(i));

					servo->set_enabled(sensors.servo[i].torqueEnabled);

					servo->set_p_gain(sensors.servo[i].pGain);
					servo->set_i_gain(sensors.servo[i].iGain);
					servo->set_d_gain(sensors.servo[i].dGain);

					servo->set_goal_position(sensors.servo[i].goalPosition);
					servo->set_goal_speed(sensors.servo[i].movingSpeed);
					servo->set_torque_limit(sensors.servo[i].torqueLimit);

					servo->set_present_position(sensors.servo[i].presentPosition);
					servo->set_present_speed(sensors.servo[i].presentSpeed);

					servo->set_load(sensors.servo[i].load);
					servo->set_voltage(sensors.servo[i].voltage);
					servo->set_temperature(sensors.servo[i].temperature);

					/*if (sensors.servo[i].errorFlags > 0) {
						std::cout << sensors.servo[i].errorFlags << std::endl;
					}*/
				}

				auto* gyro = sensorData->mutable_gyroscope();
				gyro->set_x(sensors.gyroscope.x);
				gyro->set_y(sensors.gyroscope.y);
				gyro->set_z(sensors.gyroscope.z);

				auto* accel = sensorData->mutable_accelerometer();
				accel->set_x(sensors.accelerometer.x);
				accel->set_y(sensors.accelerometer.y);
				accel->set_z(sensors.accelerometer.z);

				auto* lfsr = sensorData->mutable_left_fsr();
				lfsr->set_fsr1(sensors.fsr.left.fsr1);
				lfsr->set_fsr2(sensors.fsr.left.fsr2);
				lfsr->set_fsr3(sensors.fsr.left.fsr3);
				lfsr->set_fsr4(sensors.fsr.left.fsr4);
				lfsr->set_centre_x(sensors.fsr.left.centreX);
				lfsr->set_centre_y(sensors.fsr.left.centreY);
				
				auto* rfsr = sensorData->mutable_right_fsr();
				rfsr->set_fsr1(sensors.fsr.right.fsr1);
				rfsr->set_fsr2(sensors.fsr.right.fsr2);
				rfsr->set_fsr3(sensors.fsr.right.fsr3);
				rfsr->set_fsr4(sensors.fsr.right.fsr4);
				rfsr->set_centre_x(sensors.fsr.right.centreX);
				rfsr->set_centre_y(sensors.fsr.right.centreY);

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

				// TODO!

				/*auto* orient = sensorData->mutable_orientation();
				orient->add_float_value(0);
				orient->add_float_value(0);
				orient->add_float_value(0);*/

				send(message);
			});

			on<Trigger<Image>, With<DarwinSensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image, const DarwinSensors&) {

				//std::cout << "Image!" << std::endl;

				Message message;
				message.set_type(Message::VISION);
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
				Message message;
				message.set_type(Message::REACTION_STATISTICS);
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

				for (auto& log : stats.log) {
					reactionStatistics->add_log(log);
				}

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

				Message message;
				message.set_type(Message::VISION);
				message.set_utc_timestamp(std::time(0));
				Message::Vision* api_vision = message.mutable_vision();

				Message::VisionClassifiedImage* api_classified_image = api_vision->mutable_classified_image();

				for (auto& rowColourSegments : image.horizontal_filtered_segments.m_segmentedScans) {
					for (auto& colorSegment : rowColourSegments) {
						auto& start = colorSegment.m_start;
						auto& end = colorSegment.m_end;
						auto& colour = colorSegment.m_colour;

						Message::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
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

						Message::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
						api_segment->set_start_x(start[0]);
						api_segment->set_start_y(start[1]);
						api_segment->set_end_x(end[0]);
						api_segment->set_end_y(end[1]);
						api_segment->set_colour(colour);
					}
				}

				for (auto& matchedSegment : image.matched_vertical_segments)
				{
					for (auto& ballColumnColourSegment : matchedSegment.second)
					{
						auto& start = ballColumnColourSegment.m_start;
						auto& end = ballColumnColourSegment.m_end;
						auto& colour = ballColumnColourSegment.m_colour;
						auto& colourClass = matchedSegment.first;

						Message::VisionTransitionSegment* api_segment = api_classified_image->add_transition_segment();
						api_segment->set_start_x(start[0]);
						api_segment->set_start_y(start[1]);
						api_segment->set_end_x(end[0]);
						api_segment->set_end_y(end[1]);
						api_segment->set_colour(colour);
						api_segment->set_colour_class(colourClass);
					}
				}
	
				for (auto& matchedSegment : image.matched_horizontal_segments)
				{
					for (auto& ballRowColourSegment : matchedSegment.second)
					{
						auto& start = ballRowColourSegment.m_start;
						auto& end = ballRowColourSegment.m_end;
						auto& colour = ballRowColourSegment.m_colour;
						auto& colourClass = matchedSegment.first;

						Message::VisionTransitionSegment* api_segment = api_classified_image->add_transition_segment();
						api_segment->set_start_x(start[0]);
						api_segment->set_start_y(start[1]);
						api_segment->set_end_x(end[0]);
						api_segment->set_end_y(end[1]);
						api_segment->set_colour(colour);
						api_segment->set_colour_class(colourClass);
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
		 * pub.send need to be synchronized with a concurrency primitive
		 * (such as a mutex)
		 */
		void NUbugger::send(zmq::message_t& packet) {
			std::lock_guard<std::mutex> lock(mutex);
			pub.send(packet);
		}

		void NUbugger::send(Message message) {
			auto serialized = message.SerializeAsString();
			zmq::message_t packet(serialized.size());
			memcpy(packet.data(), serialized.data(), serialized.size());
			send(packet);
		}

	} // support
} // modules
