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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#include "VirtualCamera.h"

#include "extension/Configuration.h"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/vision/LookUpTable.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"

namespace module {
namespace support {

    using extension::Configuration;

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::LookUpTable;

    using FOURCC = utility::vision::FOURCC;

    VirtualCamera::VirtualCamera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), emitImageHandle() {

        emitImageHandle = on<Every<30, Per<std::chrono::seconds>>, With<CameraParameters>, Single>().then(
            "Simulated Images (VCamera)", [this]() {
                auto msg           = std::make_unique<message::input::Image>();
                msg->format        = FOURCC::BGGR;
                msg->camera_id     = 0;
                msg->serial_number = "VirtualCamera";
                msg->timestamp     = NUClear::clock::now();
                msg->Hcw           = Hcw;

                utility::vision::loadImage(imagePath, *msg);
                emit(msg);
            });

        on<Configuration>("VirtualLookUpTable.yaml").then([this](const Configuration& config) {
            auto lut = std::make_unique<LookUpTable>(config.config.as<LookUpTable>());
            emit(lut);
        });


        on<Configuration>("VirtualCamera.yaml")
            .then("VirtualCamera: Emit VirCam params", [this](const Configuration& config) {
                // Use configuration here from file VirtualCamera.yaml


                auto cameraParameters = std::make_unique<CameraParameters>();
                double tanHalfFOV[2], imageCentre[2];

                // Generic camera parameters
                cameraParameters->imageSizePixels << config["imageWidth"].as<uint>(), config["imageHeight"].as<uint>();
                cameraParameters->FOV << config["FOV_X"].as<double>(), config["FOV_Y"].as<double>();
                // TODO: configure the offset? probably not necessary for pinhole
                cameraParameters->centreOffset = Eigen::Vector2i::Zero();

                imagePath = config["image"]["path"].as<std::string>();

                for (size_t row = 0; row < config["image"]["Hcw"].config.size(); row++) {
                    for (size_t col = 0; col < config["image"]["Hcw"][row].config.size(); col++) {
                        Hcw(row, col) = config["image"]["Hcw"][row][col].as<double>();
                    }
                }

                if (config["lens_type"].as<std::string>().compare("pinhole") == 0) {
                    // Pinhole specific
                    cameraParameters->lens                     = CameraParameters::LensType::PINHOLE;
                    tanHalfFOV[0]                              = std::tan(cameraParameters->FOV[0] * 0.5);
                    tanHalfFOV[1]                              = std::tan(cameraParameters->FOV[1] * 0.5);
                    imageCentre[0]                             = cameraParameters->imageSizePixels[0] * 0.5;
                    imageCentre[1]                             = cameraParameters->imageSizePixels[1] * 0.5;
                    cameraParameters->pinhole.distortionFactor = config["DISTORTION_FACTOR"].as<double>();
                    cameraParameters->pinhole.pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]),
                        (tanHalfFOV[1] / imageCentre[1]);
                    cameraParameters->pinhole.focalLengthPixels = imageCentre[0] / tanHalfFOV[0];
                }
                else if (config["lens_type"].as<std::string>().compare("radial") == 0) {
                    // Radial specific
                    cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
                    cameraParameters->radial.radiansPerPixel = config["lens"]["radiansPerPixel"].as<float>();
                    cameraParameters->centreOffset = convert<int, 2>(config["lens"]["centreOffset"].as<arma::ivec>());
                }
                else {
                    log<NUClear::ERROR>("LENS TYPE UNDEFINED: choose from 'pinhole' or 'radial'");
                }

                bool emit_images = config["emit_images"].as<bool>();
                if (emit_images) {
                    emitImageHandle.enable();
                }
                else {
                    emitImageHandle.disable();
                }
                std::cout << "Emitting camera parameters from VirtualCamera" << std::endl;

                emit<Scope::DIRECT>(std::move(cameraParameters));
            });
    }
}  // namespace support
}  // namespace module
