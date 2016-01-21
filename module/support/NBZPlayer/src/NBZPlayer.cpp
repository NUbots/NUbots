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

#include "message/support/Configuration.h"
#include "message/input/Image.h"
#include "message/input/CameraParameters.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/input/proto/Image.pb.h"
#include "message/input/proto/Sensors.pb.h"

namespace module {
namespace support {

    using message::support::Configuration;
    using message::input::Image;
    using message::platform::darwin::DarwinSensors;
    using message::input::CameraParameters;

    template <typename T>
    using Serialise = NUClear::util::serialise::Serialise<T>;

    NBZPlayer::NBZPlayer(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("NBZPlayer.yaml").then([this](const Configuration& config) {

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

            // Extract the timestamp and use it for an offset
            uint64_t timestamp = *reinterpret_cast<uint64_t*>(data.data());
            offset = initialTime - NUClear::clock::time_point(std::chrono::milliseconds(timestamp));

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

            emit<Scope::DIRECT>(std::move(cameraParameters));
        });

        on<Always>().then([this] {

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
                    return;
                }

                // replay, go back to beginning of file
                input.clear();
                input.seekg(0, std::ios::beg);
                auto now = NUClear::clock::now();
                offset += now - initialTime;
                initialTime = now;
                log<NUClear::INFO>("Reached end of log file, replaying");
                return;
            }

            // Extract the timestamp and use it for an offset
            uint64_t timestamp = *reinterpret_cast<uint64_t*>(data.data());
            NUClear::clock::time_point timeToRun = NUClear::clock::time_point(std::chrono::milliseconds(timestamp)) + offset;

            // Extract the hash
            std::array<uint64_t, 2> hash = *reinterpret_cast<std::array<uint64_t, 2>*>(data.data() + sizeof(uint64_t));

            if(hash == Serialise<message::input::proto::Image>::hash()) {
                // Parse our image
                message::input::proto::Image proto;
                proto.ParsePartialFromArray(data.data() + (sizeof(uint64_t) * 3),
                                        data.size() - (sizeof(uint64_t) * 3));

                // Get the width and height
                int width = proto.dimensions().x();
                int height = proto.dimensions().y();
                const std::string& source = proto.data();

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
            else if(hash == Serialise<message::input::proto::Sensors>::hash()) {
                message::input::proto::Sensors proto;
                proto.ParsePartialFromArray(data.data() + (sizeof(uint64_t) * 3),
                                        data.size() - (sizeof(uint64_t) * 3));

                // Make a darwin sensors
                auto sensors = std::make_unique<DarwinSensors>();

                sensors->accelerometer = {
                     proto.accelerometer().y(),
                    -proto.accelerometer().x(),
                    -proto.accelerometer().z()
                };

                sensors->gyroscope = {
                    -proto.gyroscope().x(),
                    -proto.gyroscope().y(),
                     proto.gyroscope().z()
                };

                for(const auto& s : proto.servo()) {

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

                for(const auto& l : proto.led()) {
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

                for(const auto& l : proto.button()) {

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
        });
    }
}
}

