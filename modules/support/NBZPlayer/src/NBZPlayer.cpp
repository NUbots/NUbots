/*
 * This file is part of NUbots Codebase.
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

#include "NBZPlayer.h"

#include "messages/support/Configuration.h"
#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/input/Image.h"
#include "messages/input/CameraParameters.h"
#include "messages/platform/darwin/DarwinSensors.h"

namespace modules {
namespace support {

    using messages::support::Configuration;
    using messages::input::Image;
    using messages::platform::darwin::DarwinSensors;
    using messages::support::nubugger::proto::Message;
    using messages::input::CameraParameters;

    NBZPlayer::NBZPlayer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {


            on<Trigger<Configuration<NBZPlayer>>>([this](const Configuration<NBZPlayer>& config) {

                std::string path = config["file"].as<std::string>();
                replay = config["replay"].as<bool>();

                // Setup the file
                // input.push(boost::iostreams::gzip_decompressor());
                // Open a file using the file name and timestamp
                input.close();
                input.clear();
                std::string filename = config["file"].as<std::string>();
                input.open(filename, std::ios::binary);
                if (input.fail()) {
                    log<NUClear::FATAL>("NBZPlayer could not read from file:", filename);
                    return;
                }

                // Read the first 32 bit int to work out the size
                uint32_t size;
                input.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));

                // Read that much into a string
                std::vector<char> data(size);
                input.read(data.data(), size);

                // Read the message
                Message message;
                message.ParsePartialFromArray(data.data(), data.size());
                initialTime = NUClear::clock::now();
                offset = initialTime - time_t(std::chrono::milliseconds(message.utc_timestamp()));

                auto cameraParameters = std::make_unique<CameraParameters>();

                cameraParameters->imageSizePixels = { 640, 480 };
                cameraParameters->FOV = { 1.0472 , 0.785398 };
                cameraParameters->distortionFactor = -0.000018;
                arma::vec2 tanHalfFOV;
                tanHalfFOV << std::tan(cameraParameters->FOV[0] * 0.5) << std::tan(cameraParameters->FOV[1] * 0.5);
                arma::vec2 imageCentre;
                imageCentre << cameraParameters->imageSizePixels[0] * 0.5 << cameraParameters->imageSizePixels[1] * 0.5;
                cameraParameters->pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]) << (tanHalfFOV[1] / imageCentre[1]);
                cameraParameters->focalLengthPixels = imageCentre[0] / tanHalfFOV[0];


                emit<DIRECT>(std::move(cameraParameters));

            });


            powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask([this] {
                while(true) {

                    // Read the first 32 bit int to work out the size
                    uint32_t size;
                    input.read(reinterpret_cast<char*>(&size), sizeof(uint32_t));

                    // Read that much into a string
                    std::vector<char> data(size);
                    input.read(data.data(), size);

                    // If we are going to replay then reset the stream
                    if(input.eof()) {
                        if (!replay) {
                            log<NUClear::INFO>("Reached end of log file, quitting");
                            break;
                        }

                        // replay, go back to beginning of file
                        input.clear();
                        input.seekg(0, std::ios::beg);
                        auto now = NUClear::clock::now();
                        offset += now - initialTime;
                        initialTime = now;
                        log<NUClear::INFO>("Reached end of log file, replaying");
                        continue;
                    }

                    // Read the message
                    Message message;
                    message.ParsePartialFromArray(data.data(), data.size());

                    // If it's an image
                    if(message.type() == Message::IMAGE) {

                        // Work out our time to run
                        time_t timeToRun = time_t(std::chrono::milliseconds(message.utc_timestamp())) + offset;

                        // Get the width and height
                        int width = message.image().dimensions().x();
                        int height = message.image().dimensions().y();
                        const std::string& source = message.image().data();

                        // Get the image data
                        std::vector<uint8_t> pixels(source.size());

                        std::memcpy(pixels.data(), source.data(), source.size());

                        // Build the image
                        auto image = std::make_unique<Image>(width, height, NUClear::clock::now(), std::move(pixels));

                        // Wait until it's time to display it
                        std::this_thread::sleep_until(timeToRun);

                        // Send it!
                        emit(std::move(image));
                    }
                    else if(message.type() == Message::SENSOR_DATA) {

                        // Make a darwin sensors
                        auto sensors = std::make_unique<DarwinSensors>();

                        sensors->accelerometer = {

                             message.sensor_data().accelerometer().y(),
                            -message.sensor_data().accelerometer().x(),
                            -message.sensor_data().accelerometer().z()
                        };

                        sensors->gyroscope = {
                            -message.sensor_data().gyroscope().x(),
                            -message.sensor_data().gyroscope().y(),
                             message.sensor_data().gyroscope().z()
                        };

                        for(const auto& s : message.sensor_data().servo()) {

                            auto& servo = sensors->servo[s.id()];

                            servo.errorFlags = s.error_flags();
                            servo.torqueEnabled = s.enabled();

                            servo.pGain = s.p_gain();
                            servo.iGain = s.i_gain();
                            servo.dGain = s.d_gain();

                            servo.goalPosition = s.goal_position();
                            servo.movingSpeed = s.goal_velocity();

                            servo.presentPosition = s.present_position();
                            servo.presentSpeed = s.present_velocity();

                            servo.load = s.load();
                            servo.voltage = s.voltage();
                            servo.temperature = s.temperature();

                        }

                        for(const auto& l : message.sensor_data().led()) {
                            switch(l.id()) {
                                case 0: {
                                    sensors->ledPanel.led2 = l.colour() == 0xFF0000;
                                } break;
                                case 1:{
                                    sensors->ledPanel.led3 = l.colour() == 0xFF0000;
                                } break;
                                case 2:{
                                    sensors->ledPanel.led4 = l.colour() == 0xFF0000;
                                } break;
                                case 3:{
                                    sensors->eyeLED.r = (l.colour() & 0xFF0000) << 16;
                                    sensors->eyeLED.g = (l.colour() & 0x00FF00) << 8;
                                    sensors->eyeLED.b = (l.colour() & 0x0000FF) << 0;
                                } break;
                                case 4:{
                                    sensors->headLED.r = (l.colour() & 0xFF0000) << 16;
                                    sensors->headLED.g = (l.colour() & 0x00FF00) << 8;
                                    sensors->headLED.b = (l.colour() & 0x0000FF) << 0;
                                } break;
                            }

                        }

                        for(const auto& l : message.sensor_data().button()) {

                            switch(l.id()) {
                                case 0: {
                                    sensors->buttons.left = l.value();
                                } break;
                                case 1: {
                                    sensors->buttons.middle = l.value();
                                } break;
                            }
                        }

                        emit(std::move(sensors));
                    }
                }
            },
            [] {
                // This is a buggy module! recode me!
            }));

    }

}
}

