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
#include "message/vision/LookUpTable.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"
#include "utility/vision/LookUpTable.h"
#include "utility/vision/Vision.h"

namespace module {
namespace support {

    using extension::Configuration;

    using message::input::Image;
    using message::vision::LookUpTable;

    using FOURCC = utility::vision::FOURCC;

    VirtualCamera::VirtualCamera(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), emitImageHandle() {

        emitImageHandle =
            on<Every<30, Per<std::chrono::seconds>>, Single>().then("Simulated Images (VCamera)", [this]() {
                auto msg       = std::make_unique<message::input::Image>();
                msg->format    = FOURCC::BGGR;
                msg->camera_id = 0;
                msg->name      = "VirtualCamera";
                msg->timestamp = NUClear::clock::now();
                msg->Hcw       = Hcw;
                msg->lens      = lens;

                utility::vision::loadImage(imagePath, *msg);
                emit(msg);
            });

        on<Configuration>("VirtualLookUpTable.yaml").then([this](const Configuration& config) {
            auto lut = std::make_unique<LookUpTable>(config.config.as<LookUpTable>());
            emit(lut);
        });


        on<Configuration>("VirtualCamera.yaml").then([this](const Configuration& config) {
            imagePath = config["image"]["path"].as<std::string>();

            for (size_t row = 0; row < config["image"]["Hcw"].config.size(); row++) {
                for (size_t col = 0; col < config["image"]["Hcw"][row].config.size(); col++) {
                    Hcw(row, col) = config["image"]["Hcw"][row][col].as<double>();
                }
            }

            if (config["lens_type"].as<std::string>().compare("pinhole") == 0) {
                // Pinhole specific
                lens.projection = Image::Lens::Projection::RECTILINEAR;
                lens.fov << config["FOV_X"].as<float>(), config["FOV_Y"].as<float>();
                lens.focal_length = (config["imageWidth"].as<float>() * 0.5f) / std::tan(lens.fov.x() * 0.5f);
                lens.centre << 0.0f, 0.0f;
            }
            else if (config["lens_type"].as<std::string>().compare("radial") == 0) {
                // Radial specific
                lens.projection = Image::Lens::Projection::EQUIDISTANT;
                lens.fov << config["FOV_X"].as<float>(), config["FOV_Y"].as<float>();
                lens.focal_length = 1.0f / config["lens"]["radiansPerPixel"].as<float>();
                lens.centre       = convert(config["lens"]["centreOffset"].as<arma::fvec>());
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
        });
    }
}  // namespace support
}  // namespace module
