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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUsight.h"

#include "message/input/Image.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/LookUpTableDiff.h"
#include "message/vision/VisionObjects.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::CameraParameters;
    using message::input::Image;
    using message::vision::Balls;
    using message::vision::ClassifiedImage;
    using message::vision::Goals;
    using message::vision::Line;
    using message::vision::LookUpTableDiff;
    using message::vision::NUsightBalls;
    using message::vision::NUsightGoals;
    using message::vision::NUsightLines;
    using message::vision::NUsightObstacles;
    using message::vision::Obstacle;

    void NUsight::provideVision() {
        handles["camera_parameters"].push_back(on<Every<1, Per<std::chrono::seconds>>, With<CameraParameters>>().then(
            [this](std::shared_ptr<const CameraParameters> cameraParameters) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(cameraParameters), "nusight", false);
            }));

        handles["image"].push_back(
            on<Trigger<Image>, Single, Priority::LOW>().then([this](std::shared_ptr<const Image> image) {
                if (NUClear::clock::now() - last_image < max_image_duration) {
                    return;
                }

                powerplant.emit_shared<Scope::NETWORK>(std::move(image), "nusight", false);

                last_image = NUClear::clock::now();
            }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage>, Single, Priority::LOW>().then(
            [this](std::shared_ptr<const ClassifiedImage> image) {
                if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                    return;
                }

                powerplant.emit_shared<Scope::NETWORK>(std::move(image), "nusight", false);

                last_classified_image = NUClear::clock::now();
            }));

        handles["vision_object"].push_back(on<Trigger<Balls>, Single, Priority::LOW>().then([this](const Balls& balls) {
            auto nusight = std::make_unique<Balls>();
            emit<Scope::NETWORK>(nusight, "nusight", false);
        }));

        handles["vision_object"].push_back(on<Trigger<Goals>, Single, Priority::LOW>().then([this](const Goals& goals) {
            auto nusight = std::make_unique<Goals>();
            emit<Scope::NETWORK>(nusight, "nusight", false);
        }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then(
            [this](std::shared_ptr<const LookUpTableDiff> tableDiff) {
                powerplant.emit_shared<Scope::NETWORK>(std::move(tableDiff), "nusight", true);
            }));
    }
}  // namespace support
}  // namespace module
