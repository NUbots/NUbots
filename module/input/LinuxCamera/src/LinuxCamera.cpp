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

#include "LinuxCamera.h"

extern "C" {
    #include <jpeglib.h>
}

#include "V4L2Camera.h"
#include "message/input/Image.h"
#include "message/input/CameraParameters.h"
#include "message/support/Configuration.h"

namespace module {
    namespace input {

        using message::support::Configuration;
        using message::input::CameraParameters;
        using message::input::Image;

        // We assume that the device will always be video0, if not then change this
        LinuxCamera::LinuxCamera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
            on<Every<V4L2Camera::FRAMERATE, Per<std::chrono::seconds>>, Single>().then("Read Camera", [this] {

                // If the camera is ready, get an image and emit it
                if (camera.isStreaming()) {
                    emit(std::make_unique<Image>(camera.getImage()));
                }
            });

            // When we shutdown, we must tell our camera class to close (stop streaming)
            on<Shutdown>().then([this] {
                camera.closeCamera();
            });

            on<Configuration>("LinuxCamera.yaml").then([this](const Configuration& config) {

                auto cameraParameters = std::make_unique<CameraParameters>();

                cameraParameters->imageSizePixels << config["imageWidth"].as<uint>() << config["imageHeight"].as<uint>();
                cameraParameters->FOV << config["FOV_X"].as<double>() << config["FOV_Y"].as<double>();
                cameraParameters->distortionFactor = config["DISTORTION_FACTOR"].as<double>();
                arma::vec2 tanHalfFOV;
                tanHalfFOV << std::tan(cameraParameters->FOV[0] * 0.5) << std::tan(cameraParameters->FOV[1] * 0.5);
                arma::vec2 imageCentre;
                imageCentre << cameraParameters->imageSizePixels[0] * 0.5 << cameraParameters->imageSizePixels[1] * 0.5;
                cameraParameters->pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]) << (tanHalfFOV[1] / imageCentre[1]);
                cameraParameters->focalLengthPixels = imageCentre[0] / tanHalfFOV[0];


                emit<Scope::DIRECT>(std::move(cameraParameters));

                try {
                    // Recreate the camera device at the required resolution
                    int width = config["imageWidth"].as<uint>();
                    int height = config["imageHeight"].as<uint>();
                    std::string deviceID = config["deviceID"].as<std::string>();
                    std::string format = config["imageFormat"].as<std::string>();

                    if (camera.getWidth() != static_cast<size_t>(width)
                        || camera.getHeight() != static_cast<size_t>(height)
                        || camera.getFormat() != format
                        || camera.getDeviceID() != deviceID) {
                        camera.resetCamera(deviceID, format, width, height);
                    }

                    // Set all other camera settings
                    for(auto& setting : config.config) {
                        auto& settings = camera.getSettings();
                        auto it = settings.find(setting.first.as<std::string>());
                        if(it != settings.end()) {
                            if(it->second.set(setting.second.as<int>()) == false) {
                                NUClear::log<NUClear::DEBUG>("Failed to set " + it->first + " on camera");
                            }
                        }
                    }

                    // Start the camera streaming video
                    camera.startStreaming();
                } catch(const std::exception& e) {
                    NUClear::log<NUClear::DEBUG>(std::string("Exception while setting camera configuration: ") + e.what());
                    throw e;
                }
            });

            on<Configuration, Every<1, std::chrono::seconds>>("LinuxCamera.yaml").then("Camera Setting Applicator", [this] (const Configuration& config) {
                if(camera.isStreaming()) {
                    // Set all other camera settings
                    for(auto& setting : config.config) {
                        auto& settings = camera.getSettings();
                        auto it = settings.find(setting.first.as<std::string>());
                        if(it != settings.end()) {
                            if(it->second.set(setting.second.as<int>()) == false) {
                                NUClear::log<NUClear::DEBUG>("Failed to set " + it->first + " on camera");
                            }
                        }
                    }
                }
            });
        }

    }  // input
}  // modules
