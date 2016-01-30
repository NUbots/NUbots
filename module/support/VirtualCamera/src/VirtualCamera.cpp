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

#include "message/support/Configuration.h"
#include "message/input/CameraParameters.h"

namespace module {
namespace support {

    using message::support::Configuration;
    using message::input::CameraParameters;

    VirtualCamera::VirtualCamera(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("VirtualCamera.yaml").then("VirtualCamera: Emit VirCam params",[this] (const Configuration& config) {
            // Use configuration here from file VirtualCamera.yaml

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

            std::cout << "Emitting camera parameters from VirtualCamera" << std::endl;

            emit<Scope::DIRECT>(std::move(cameraParameters));

        });
    }
}
}
