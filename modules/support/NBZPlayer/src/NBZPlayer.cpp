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
#include "messages/vision/LookUpTable.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "utility/nubugger/NUHelpers.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/geometry/Quad.h"

namespace modules {
namespace support {

    using messages::support::Configuration;
    using messages::input::Image;
    using messages::platform::darwin::DarwinSensors;
    using messages::support::nubugger::proto::Message;
    using messages::input::CameraParameters;
    using messages::vision::LookUpTable;
    using messages::vision::Colour;
    using utility::nubugger::graph;
    using messages::support::SaveConfiguration;
    using utility::math::geometry::Circle;
    using utility::math::geometry::Quad;

    arma::uvec3 colourForTime(const uint64_t& timestamp) {

        double r, g, b;

        const double h = (sin(2.0 * M_PI * double(timestamp) / 100000.0) + 1.0) / 2.0;
        const double s = 1.0;
        const double v = 1.0;

        int i = int(h * 6.0);
        double f = h * 6.0 - i;
        double p = v * (1.0 - s);
        double q = v * (1.0 - f * s);
        double t = v * (1.0 - (1.0 - f) * s);
        switch (i % 6) {
            case 0: r = v, g = t, b = p; break;
            case 1: r = q, g = v, b = p; break;
            case 2: r = p, g = v, b = t; break;
            case 3: r = p, g = q, b = v; break;
            case 4: r = t, g = p, b = v; break;
            case 5: r = v, g = p, b = q; break;
        }

        return { uint(r * 255), uint(g * 255), uint(b * 255) };
    }

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

                bool atStartColour = false;
                uint64_t lastTime = 0;
                Message imageMessage;
                while(true) {

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
                    std::cout << message.type() << std::endl;
                    if(lastTime != 0) {
                        const double h1 = (sin(2.0 * M_PI * double(lastTime) / 100000.0) + 1.0) / 2.0;
                        const double h2 = (sin(2.0 * M_PI * double(message.utc_timestamp()) / 100000.0) + 1.0) / 2.0;

                        const double target = 60.0/360.0;

                        if(h1 <= target && h2 >= target) {
                            atStartColour = true;
                        }
                        if(message.type() == Message::IMAGE) {
                            imageMessage = message;
                            break;
                        }
                    }

                    lastTime = message.utc_timestamp();

                }

                 // Get the width and height
                int width = imageMessage.image().dimensions().x();
                int height = imageMessage.image().dimensions().y();
                const std::string& source = imageMessage.image().data();

                // Get the image data
                std::vector<Image::Pixel> pixels(source.size() / 3);

                std::memcpy(pixels.data(), source.data(), source.size());

                // Build the image
                Image image(width, height, std::move(pixels));

                LookUpTable lut(6, 7, 7, std::vector<Colour>(1 << (6 + 7 + 7), Colour::UNCLASSIFIED));

                // Ball!
                auto ballCircle = Circle(9.0, arma::vec2({181, 196}));
                // find the min and max y points on the circle
                // capped at the bounds of the image
                uint minY = std::max(std::ceil(ballCircle.centre[1] - ballCircle.radius), 0.0);
                uint maxY = std::min(std::floor(ballCircle.centre[1] + ballCircle.radius), double(image.height() - 1));

                // loop through pixels on the image in bounding box
                for (uint y = minY + 1; y <= maxY - 1; ++y) {
                    auto edgePoints = ballCircle.getEdgePoints(y);
                    uint minX = std::max(edgePoints[0], 0.0);
                    uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                    for (uint x = minX + 1; x <= maxX - 1; ++x) {
                        lut(image(x, y)) = Colour::ORANGE;
                    }
                }

                // Do our goals!
                for(auto quad : std::vector<Quad>({
                    Quad(arma::vec2({51,58}), arma::vec2({54,195}), arma::vec2({71,194}), arma::vec2({68,57})),
                    Quad(arma::vec2({269,71}), arma::vec2({271,184}), arma::vec2({285,184}), arma::vec2({286,71}))
                })) {
                    uint minY = std::max(std::min(quad.getBottomLeft()[1], quad.getBottomRight()[1]), 0.0);
                    uint maxY = std::min(std::max(quad.getTopLeft()[1], quad.getTopRight()[1]), double(image.height() - 1));

                    for (uint y = minY + 1; y <= maxY - 1; ++y) {
                        arma::vec2 edgePoints;
                        try {
                            edgePoints = quad.getEdgePoints(y);
                        } catch (std::domain_error&) {
                            continue; // no intersection
                        }
                        uint minX = std::max(edgePoints[0], 0.0);
                        uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                        for (uint x = minX + 1; x <= maxX - 1; ++x) {
                            lut(image(x, y)) = Colour::YELLOW;
                        }
                    }

                }

                // Do our field!
                for(auto quad : std::vector<Quad>({
                    Quad(arma::vec2({0,160}), arma::vec2({0,192}), arma::vec2({46,186}), arma::vec2({46,159})),
                    Quad(arma::vec2({77,161}), arma::vec2({76,184}), arma::vec2({233,177}), arma::vec2({222,154})),
                    Quad(arma::vec2({0,203}), arma::vec2({0,226}), arma::vec2({163,213}), arma::vec2({156,193})),
                    Quad(arma::vec2({197,191}), arma::vec2({198,210}), arma::vec2({288,205}), arma::vec2({267,189})),
                    Quad(arma::vec2({0,234}), arma::vec2({0,239}), arma::vec2({257,239}), arma::vec2({257,217}))
                })) {
                    uint minY = std::max(std::min(quad.getBottomLeft()[1], quad.getBottomRight()[1]), 0.0);
                    uint maxY = std::min(std::max(quad.getTopLeft()[1], quad.getTopRight()[1]), double(image.height() - 1));

                    for (uint y = minY + 1; y <= maxY - 1; ++y) {
                        arma::vec2 edgePoints;
                        try {
                            edgePoints = quad.getEdgePoints(y);
                        } catch (std::domain_error&) {
                            continue; // no intersection
                        }
                        uint minX = std::max(edgePoints[0], 0.0);
                        uint maxX = std::min(edgePoints[1], double(image.width() - 1));

                        for (uint x = minX + 1; x <= maxX - 1; ++x) {
                            lut(image(x, y)) = Colour::GREEN;
                        }
                    }

                }

                // Save this LUT
                emit<Scope::DIRECT>(std::make_unique<SaveConfiguration>(SaveConfiguration{ "LookUpTable.yaml", YAML::Node(lut) }));

                // Print our timestamp!
                emit(std::make_unique<NUClear::clock::duration>(offset));

                // Keep doing this until we pass our "gate" colour

                auto cameraParameters = std::make_unique<CameraParameters>();

                cameraParameters->imageSizePixels = { 320, 240 };
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
                            exit(0);
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

                        auto colour = colourForTime(message.utc_timestamp());

                        // Get the width and height
                        int width = message.image().dimensions().x();
                        int height = message.image().dimensions().y();
                        const std::string& source = message.image().data();

                        // Get the image data
                        std::vector<Image::Pixel> pixels(source.size() / 3);

                        std::memcpy(pixels.data(), source.data(), source.size());

                        // Build the image
                        auto image = std::make_unique<Image>(width, height, std::move(pixels));

                        // Wait until it's time to display it
                        std::this_thread::sleep_until(timeToRun);

                        // Emit the graph of the colours
                        emit(graph("colour", colour));

                        // Send it!
                        emit(std::move(image));
                    }
                    else if(message.type() == Message::SENSOR_DATA) {

                        // Make a darwin sensors
                        auto sensors = std::make_unique<DarwinSensors>();

                        sensors->accelerometer = {
                            -message.sensor_data().accelerometer().y(),
                             message.sensor_data().accelerometer().x(),
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

